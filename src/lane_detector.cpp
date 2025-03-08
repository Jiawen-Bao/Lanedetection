#include "lane_detector.h"

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
        if (std::abs(slope) > 0.3) {
            if (mid_x < img_center_x && slope < 0) {
                left_lines.push_back(line);  // 左车道线 (负斜率)
            }
            else if (mid_x > img_center_x && slope > 0) {
                right_lines.push_back(line); // 右车道线 (正斜率)
            }
        }
    }
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
    cv::HoughLinesP(masked_edges, lines, 1, CV_PI / 180, 80, 30, 10);

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
    return result;
}
