#pragma once

#include "detections_pkg/types.h"
#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
#include <array>
#include <memory>

namespace detections {

/// Two-stage hazmat detector using ONNX Runtime:
///   Stage 1: YOLOv8 detects and localizes hazmat signs
///   Stage 2: ResNet-18 classifies the type of hazmat material
class HazmatDetector {
public:
    HazmatDetector();
    ~HazmatDetector();

    /// Load ONNX models. Returns true on success.
    bool initialize(const std::string& yoloPath,
                    const std::string& resnetPath,
                    bool useCUDA);

    /// Process a BGR frame. Returns detected hazmat signs.
    /// interval controls how often detection runs (every N calls).
    std::vector<HazmatDetection> detect(const cv::Mat& bgrFrame,
                                        int interval,
                                        float confThreshold,
                                        float nmsThreshold);

    bool isLoaded() const { return m_loaded; }

private:
    // Stage 1: YOLO detection
    std::vector<HazmatDetection> runYOLO(const cv::Mat& bgrFrame,
                                         float confThreshold,
                                         float nmsThreshold);

    // Stage 2: ResNet-18 classification on a crop
    void classifyCrop(const cv::Mat& crop, HazmatDetection& det);

    // YOLO preprocessing: letterbox resize + normalize
    cv::Mat preprocessYOLO(const cv::Mat& bgrFrame,
                           float& scaleX, float& scaleY,
                           int& padLeft, int& padTop);

    // NMS
    static void nms(std::vector<HazmatDetection>& dets, float nmsThresh);

    Ort::Env m_env;
    Ort::SessionOptions m_sessionOpts;
    std::unique_ptr<Ort::Session> m_yoloSession;
    std::unique_ptr<Ort::Session> m_resnetSession;

    // YOLO IO names
    std::vector<const char*> m_yoloInputNames;
    std::vector<const char*> m_yoloOutputNames;
    std::vector<std::string> m_yoloInputNamesStorage;
    std::vector<std::string> m_yoloOutputNamesStorage;

    // ResNet IO names
    std::vector<const char*> m_resnetInputNames;
    std::vector<const char*> m_resnetOutputNames;
    std::vector<std::string> m_resnetInputNamesStorage;
    std::vector<std::string> m_resnetOutputNamesStorage;

    static constexpr int YOLO_INPUT_SIZE = 640;
    static constexpr int RESNET_INPUT_SIZE = 224;
    static constexpr int NUM_CLASSES = 10;

    static constexpr std::array<const char*, NUM_CLASSES> CLASS_NAMES = {
        "Combustible", "Corrosive", "Explosives", "Flammable",
        "Flammable_gas", "Fuel", "Oxidizer", "Peroxide",
        "Poison", "Radioactive"
    };

    // ImageNet normalization
    static constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
    static constexpr float STD[3]  = {0.229f, 0.224f, 0.225f};

    bool m_loaded = false;
    int m_frameCount = 0;
    std::vector<HazmatDetection> m_cached;
};

} // namespace detections
