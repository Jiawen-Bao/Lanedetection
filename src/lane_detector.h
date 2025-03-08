#ifndef LANE_DETECTOR_H
#define LANE_DETECTOR_H

#pragma once
#include <opencv2/opencv.hpp>

class LaneDetector {
public:
    LaneDetector() = default;
    cv::Mat process(const cv::Mat& input);

private:
    void separateLines(const std::vector<cv::Vec4i>& lines,
        std::vector<cv::Vec4i>& left_lines,
        std::vector<cv::Vec4i>& right_lines,
        int img_center_x);
};

#endif // LANE_DETECTOR_H