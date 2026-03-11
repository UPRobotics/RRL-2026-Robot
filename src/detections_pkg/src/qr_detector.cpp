#include "detections_pkg/qr_detector.h"
#include <opencv2/imgproc.hpp>

namespace detections {

QRDetector::QRDetector() {
    m_scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

QRDetector::~QRDetector() = default;

std::vector<QRDetection> QRDetector::detect(const cv::Mat& bgrFrame, int interval) {
    m_frameCount++;
    if (m_frameCount % interval != 0) {
        return m_cached;
    }

    std::vector<QRDetection> results;

    // Convert to grayscale for zbar
    cv::Mat gray;
    cv::cvtColor(bgrFrame, gray, cv::COLOR_BGR2GRAY);

    zbar::Image image(gray.cols, gray.rows, "Y800",
                      gray.data, gray.cols * gray.rows);

    m_scanner.scan(image);

    for (auto it = image.symbol_begin(); it != image.symbol_end(); ++it) {
        QRDetection det;
        det.data = it->get_data();

        int n = it->get_location_size();
        for (int i = 0; i < n; i++) {
            det.points.emplace_back(it->get_location_x(i),
                                    it->get_location_y(i));
        }
        results.push_back(std::move(det));
    }

    image.set_data(nullptr, 0);
    m_cached = results;
    return m_cached;
}

} // namespace detections
