#pragma once
#include <opencv2/opencv.hpp>

class LaneDetector {
public:
    cv::Mat process(const cv::Mat& input);
};
