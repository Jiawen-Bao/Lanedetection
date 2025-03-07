#include <opencv2/opencv.hpp>
#include "lane_detector.h"

int main() {
    cv::VideoCapture cap("resources/test_video.mp4");
    if (!cap.isOpened()) {
        std::cerr << "Error opening video!" << std::endl;
        return -1;
    }

    LaneDetector detector;
    cv::Mat frame;
    while (cap.read(frame)) {
        cv::Mat result = detector.process(frame);
        cv::imshow("Lane Detection", result);
        if (cv::waitKey(30) == 27) break; // ESC 退出
    }
    return 0;
}
