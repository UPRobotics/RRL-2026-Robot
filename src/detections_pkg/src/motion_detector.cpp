#include "detections_pkg/motion_detector.h"
#include <opencv2/imgproc.hpp>

namespace detections {

MotionDetector::MotionDetector() = default;

std::vector<MotionBox> MotionDetector::detect(
    const cv::Mat& bgrFrame, int skipFrames, int minArea,
    double threshold, double weight)
{
    m_frameCount++;

    cv::Mat gray;
    cv::cvtColor(bgrFrame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

    if (m_avg.empty()) {
        gray.convertTo(m_avg, CV_32F);
        return {};
    }

    if (m_frameCount % skipFrames != 0) {
        return m_cached;
    }

    cv::accumulateWeighted(gray, m_avg, weight);

    cv::Mat avg8u;
    cv::convertScaleAbs(m_avg, avg8u);

    cv::Mat delta;
    cv::absdiff(gray, avg8u, delta);

    cv::Mat thresh;
    cv::threshold(delta, thresh, threshold, 255, cv::THRESH_BINARY);
    cv::dilate(thresh, thresh, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<MotionBox> results;
    for (auto& c : contours) {
        if (cv::contourArea(c) < minArea) continue;
        cv::Rect r = cv::boundingRect(c);
        results.push_back({r.x, r.y, r.width, r.height});
    }

    m_cached = results;
    return m_cached;
}

} // namespace detections
