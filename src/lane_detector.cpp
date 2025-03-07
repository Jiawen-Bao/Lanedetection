#include "lane_detector.h"

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

    for (auto& line : lines) {
        cv::line(result, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]),
                 cv::Scalar(0, 255, 0), 3);
    }
    return result;
}
