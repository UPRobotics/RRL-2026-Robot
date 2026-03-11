#pragma once

#include "detections_pkg/types.h"
#include <opencv2/core.hpp>
#include <vector>

namespace detections {

/// Detects motion via accumulated weighted average background subtraction.
class MotionDetector {
public:
    MotionDetector();

    /// Process a BGR frame. Returns motion bounding boxes.
    std::vector<MotionBox> detect(const cv::Mat& bgrFrame,
                                  int skipFrames,
                                  int minArea,
                                  double threshold,
                                  double weight);

    /// Reset accumulated background (e.g. after rotation change)
    void reset();

private:
    cv::Mat m_avg;          // running average (float)
    int m_frameCount = 0;
    std::vector<MotionBox> m_cached;
};

} // namespace detections
