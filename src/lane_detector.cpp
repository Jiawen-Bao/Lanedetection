#include "lane_detector.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

void LaneDetector::separateLines(const std::vector<cv::Vec4i>& lines,
    std::vector<cv::Vec4i>& left_lines,
    std::vector<cv::Vec4i>& right_lines,
    int img_center_x) {
    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];

        // 计算斜率 (m = Δy / Δx)
        double slope = (y2 - y1) * 1.0 / (x2 - x1 + 1e-6); // 避免除零错误

        // 计算线段的中点坐标
        int mid_x = (x1 + x2) / 2;

        // 过滤掉几乎水平的线
        if (std::abs(slope) > 0.1) {
            if (mid_x < img_center_x && slope < 0) {
                left_lines.push_back(line);  // 左车道线 (负斜率)
            }
            else if (mid_x > img_center_x && slope > 0) {
                right_lines.push_back(line); // 右车道线 (正斜率)
            }
        }

        if (slope < -0.3 && x1 < img_center_x && x2 < img_center_x) {
            left_lines.push_back(line);
        }


        //std::cout << "Left lines detected: " << left_lines.size() << std::endl;
        //std::cout << "Right lines detected: " << right_lines.size() << std::endl;

    }
}

bool shouldUseStraightLine(const std::vector<cv::Point>& points) {
    if (points.size() < 6) return true; // 点数太少，直接用直线

    double avg_spacing = 0;
    for (size_t i = 1; i < points.size(); i++) {
        avg_spacing += cv::norm(points[i] - points[i - 1]);  // 计算相邻点间距
    }
    avg_spacing /= (points.size() - 1);

    if (avg_spacing > 50) {  // 平均间隔过大，说明是虚线，适合直线拟合
        return true;
    }
    return false;
}

cv::Vec4f fitLineRANSAC(const std::vector<cv::Point>& points) {
    cv::Vec4f line_params;
    if (points.size() < 2) {
        std::cout << "❌ Error: Not enough points for RANSAC line fitting!" << std::endl;
        return line_params;
    }

    cv::fitLine(points, line_params, cv::DIST_L2, 0, 0.01, 0.01);

    std::cout << "✅ RANSAC Line Params: "
        << "vx: " << line_params[0] << ", vy: " << line_params[1]
        << ", x0: " << line_params[2] << ", y0: " << line_params[3] << std::endl;
    return line_params;
}


void drawStraightLane(const cv::Vec4f& line_params, cv::Mat& frame, cv::Scalar color) {
    int y1 = frame.rows;         // 画到底部
    int y2 = frame.rows * 0.6;   // 画到中间

    if (line_params[1] == 0) {
        std::cout << "❌ Warning: Invalid line parameters! Skipping drawing." << std::endl;
        return;
    }

    int x1 = static_cast<int>(line_params[2] + (y1 - line_params[3]) * line_params[0] / line_params[1]);
    int x2 = static_cast<int>(line_params[2] + (y2 - line_params[3]) * line_params[0] / line_params[1]);

    // 确保 x1, x2 在图像范围内
    x1 = std::max(0, std::min(x1, frame.cols - 1));
    x2 = std::max(0, std::min(x2, frame.cols - 1));

    std::cout << "🖌 Drawing right lane: (" << x1 << ", " << y1 << ") -> (" << x2 << ", " << y2 << ")" << std::endl;

    // **绘制白色参考线，确保绘图执行**
    cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), 3);

    // **绘制红色车道线**
    cv::line(frame, cv::Point(x1, y1), cv::Point(x2, y2), color, 3);
}




// 使用二次多项式拟合车道线
cv::Mat LaneDetector::fitPolynomial(const std::vector<cv::Point>& points, const cv::Size& img_size, cv::Scalar color) {
    if (points.size() < 5) {
        std::cout << "Warning: Not enough points for fitting!" << std::endl;
        return cv::Mat();
    }
    // 数据不足，不进行拟合

        // **计算 x 均值**
    double mean_x = 0;
    for (const auto& p : points) {
        mean_x += p.x;
    }
    mean_x /= points.size();

    // **过滤掉偏离均值过远的点（避免曲线抖动）**
    std::vector<cv::Point> filtered_points;
    for (const auto& p : points) {
        if (std::abs(p.x - mean_x) < 250) {  // 只保留靠近均值的点
            filtered_points.push_back(p);
        }
    }

    if (filtered_points.size() < 5) {
        std::cout << "Warning: Not enough filtered points for fitting!" << std::endl;
        return cv::Mat();
    }

    // 构造 A 矩阵 和 B 向量，用于最小二乘拟合 ax^2 + bx + c
    cv::Mat A(static_cast<int>(points.size()), 3, CV_64F);
    cv::Mat B(static_cast<int>(points.size()), 1, CV_64F);


    for (size_t i = 0; i < points.size(); ++i) {
        double x = points[i].x;
        A.at<double>(i, 0) = x * x;
        A.at<double>(i, 1) = x;
        A.at<double>(i, 2) = 1.0;
        B.at<double>(i, 0) = points[i].y;
    }


    // 求解最小二乘法 Ax = B，得到系数 (a, b, c)
    cv::Mat coeffs;
    cv::solve(A, B, coeffs, cv::DECOMP_SVD);

    // 提取系数
    double a = coeffs.at<double>(0, 0);
    double b = coeffs.at<double>(1, 0);
    double c = coeffs.at<double>(2, 0);

    // 设定合理的 y 范围（防止车道线延伸到天上）
    int y_min = img_size.height * 0.6;  // 只画到图像的中部
    int y_max = img_size.height - 10; // 车道线最低点


    // 生成曲线点
    std::vector<cv::Point> curve_points;
    for (int y = y_max; y >= y_min; y -= 10) {
        double delta = b * b - 4 * a * (c - y);
        if (delta < 0) {
            std::cout << "Warning: Delta < 0 for y=" << y << std::endl;
            continue;
        }

        double x1 = (-b + sqrt(delta)) / (2 * a);
        double x2 = (-b - sqrt(delta)) / (2 * a);

        // **确保 x 在图像范围内**
        if (x1 >= 0 && x1 < img_size.width) {
            curve_points.emplace_back(static_cast<int>(x1), y);
        }
        if (x2 >= 0 && x2 < img_size.width) {
            curve_points.emplace_back(static_cast<int>(x2), y);
        }
    }

    std::cout << "Curve points size: " << curve_points.size() << std::endl;

    // **检查 curve_points 是否为空**
    // **如果 `curve_points` 为空，回退到直线拟合**
    if (curve_points.empty()) {
        std::cout << "Warning: No curve points generated! Falling back to linear fitting." << std::endl;

        cv::Mat line_coeffs;
        cv::fitLine(filtered_points, line_coeffs, cv::DIST_L2, 0, 0.01, 0.01);

        double slope = line_coeffs.at<float>(1) / line_coeffs.at<float>(0);
        double intercept = line_coeffs.at<float>(3) - slope * line_coeffs.at<float>(2);

        for (int y = y_max; y >= y_min; y -= 10) {
            int x = static_cast<int>((y - intercept) / slope);
            if (x >= 0 && x < img_size.width) {
                curve_points.emplace_back(x, y);
            }
        }
    }
    // **如果还是空，返回空图像**
    if (curve_points.empty()) {
        std::cout << "Warning: Still no curve points, returning empty matrix!" << std::endl;
        return cv::Mat();
    }

    // 创建图像并绘制曲线
    cv::Mat output = cv::Mat::zeros(img_size, CV_8UC3);
    if (!curve_points.empty()) {
        cv::polylines(output, curve_points, false, color, 3);
    }
    return output;
}

cv::Mat LaneDetector::process(const cv::Mat& input) {
    cv::Mat gray, blur, edges, result;
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);
    cv::Canny(blur, edges, 50, 150);

    // 定义感兴趣区域掩膜
    cv::Mat mask = cv::Mat::zeros(edges.size(), edges.type());

    // 设定多边形顶点（这里是梯形区域）
    std::vector<cv::Point> roi_corners = {
    cv::Point(0, edges.rows),
    cv::Point(edges.cols / 2 - 50, edges.rows / 2 + 50),
    cv::Point(edges.cols / 2 + 50, edges.rows / 2 + 50),
    cv::Point(edges.cols, edges.rows)
    };

    // 填充多边形区域为白色
    cv::fillConvexPoly(mask, roi_corners, cv::Scalar(255));
    //cv::imshow("ROI Mask", mask);
    //展示ROI


    // 应用掩膜
    cv::Mat masked_edges;
    cv::bitwise_and(edges, mask, masked_edges);

    result = input.clone();

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(masked_edges, lines, 1, CV_PI / 180, 50, 20, 100);

    // 车道线分离
    std::vector<cv::Vec4i> left_lines, right_lines;
    int img_center_x = input.cols / 2;
    separateLines(lines, left_lines, right_lines, img_center_x);

    // 画出左右车道线（不同颜色）
    for (auto& line : left_lines) {
        cv::line(result, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
            cv::Scalar(255, 0, 0), 3); // 左车道线（蓝色）
    }
    for (auto& line : right_lines) {
        cv::line(result, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
            cv::Scalar(0, 0, 255), 3); // 右车道线（红色）
    }

    // 提取散点
    std::vector<cv::Point> left_points, right_points;
    for (const auto& line : left_lines) {
        left_points.emplace_back(line[0], line[1]);
        left_points.emplace_back(line[2], line[3]);
    }
    for (const auto& line : right_lines) {
        right_points.emplace_back(line[0], line[1]);
        right_points.emplace_back(line[2], line[3]);
    }

    std::cout << "Left points: " << left_points.size() << ", Right points: " << right_points.size() << std::endl;

    // **如果某侧车道线点过少，则不进行拟合**
    if (left_points.size() < 5) {
        std::cout << "Warning: Not enough left points for fitting!" << std::endl;
    }
    if (right_points.size() < 5) {
        std::cout << "Warning: Not enough right points for fitting!" << std::endl;
    }


    // **拟合曲线**
    cv::Mat left_curve, right_curve;
    if (left_points.size() >= 5) {
        left_curve = fitPolynomial(left_points, input.size(), cv::Scalar(255, 0, 0));  // 蓝色
    }
    // **处理右车道线**
    if (right_points.size() >= 5) {
        if (shouldUseStraightLine(right_points)) {
            std::cout << "Using straight line for right lane." << std::endl;
            cv::Vec4f right_lane = fitLineRANSAC(right_points);
            drawStraightLane(right_lane, result, cv::Scalar(0, 0, 255));  // **画红色直线**
        }
        else {
            std::cout << "Using curve fitting for right lane." << std::endl;
            right_curve = fitPolynomial(right_points, input.size(), cv::Scalar(0, 0, 255)); // 右侧曲线
        }
    }

    // **如果曲线拟合失败，强制使用直线**
    if (right_curve.empty() && right_points.size() >= 5) {
        std::cout << "⚠️ Right curve fitting failed, falling back to straight line!" << std::endl;
        cv::Vec4f right_lane = fitLineRANSAC(right_points);

        // **确保绘制在最终 result 图像上**
        drawStraightLane(right_lane, result, cv::Scalar(0, 0, 255));
    }


    // **确保曲线不为空**
    if (left_curve.empty()) {
        std::cout << "Warning: Left curve fitting failed!" << std::endl;
    }
    if (right_curve.empty()) {
        std::cout << "Warning: Right curve fitting failed!" << std::endl;
    }

    // 叠加到原始图像
    result = input.clone();  // 复制原始图像
    if (!left_curve.empty()) cv::addWeighted(result, 1, left_curve, 1, 0, result);
    if (!right_curve.empty()) cv::addWeighted(result, 1, right_curve, 1, 0, result);

    // **确保直线绘制在最终图像**
    cv::Vec4f right_lane = fitLineRANSAC(right_points);
    drawStraightLane(right_lane, result, cv::Scalar(0, 0, 255));


    std::cout << "Filtered Left points: " << left_points.size() << ", Filtered Right points: " << right_points.size() << std::endl;
    if (left_points.size() < 5) std::cout << "⚠️ Warning: Too few left points for fitting!" << std::endl;
    if (right_points.size() < 5) std::cout << "⚠️ Warning: Too few right points for fitting!" << std::endl;



    return result;
}
