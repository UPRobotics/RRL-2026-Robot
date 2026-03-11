#pragma once

#include "detections_pkg/types.h"
#include <zbar.h>
#include <vector>

namespace detections {

/// Runs QR code detection on BGR frames using zbar.
class QRDetector {
public:
    QRDetector();
    ~QRDetector();

    /// Process a BGR frame. Returns detected QR codes.
    /// interval controls how often detection runs (every N calls).
    std::vector<QRDetection> detect(const cv::Mat& bgrFrame, int interval);

private:
    zbar::ImageScanner m_scanner;
    int m_frameCount = 0;
    std::vector<QRDetection> m_cached;
};

} // namespace detections
