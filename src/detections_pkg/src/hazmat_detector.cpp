#include "detections_pkg/hazmat_detector.h"
#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <numeric>

namespace detections {

HazmatDetector::HazmatDetector()
    : m_env(ORT_LOGGING_LEVEL_WARNING, "HazmatDetector")
{}

HazmatDetector::~HazmatDetector() = default;

bool HazmatDetector::initialize(const std::string& yoloPath,
                                const std::string& resnetPath,
                                bool useCUDA) {
    try {
        m_sessionOpts.SetIntraOpNumThreads(2);
        m_sessionOpts.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_ALL);

        if (useCUDA) {
            OrtCUDAProviderOptions cudaOpts{};
            cudaOpts.device_id = 0;
            m_sessionOpts.AppendExecutionProvider_CUDA(cudaOpts);
            spdlog::info("CUDA execution provider requested");
        }

        // Load YOLO model
        spdlog::info("Loading YOLO model: {}", yoloPath);
        m_yoloSession = std::make_unique<Ort::Session>(
            m_env, yoloPath.c_str(), m_sessionOpts);

        // Cache YOLO I/O names
        Ort::AllocatorWithDefaultOptions alloc;
        for (size_t i = 0; i < m_yoloSession->GetInputCount(); i++) {
            auto name = m_yoloSession->GetInputNameAllocated(i, alloc);
            m_yoloInputNamesStorage.push_back(name.get());
        }
        for (size_t i = 0; i < m_yoloSession->GetOutputCount(); i++) {
            auto name = m_yoloSession->GetOutputNameAllocated(i, alloc);
            m_yoloOutputNamesStorage.push_back(name.get());
        }
        for (auto& s : m_yoloInputNamesStorage)  m_yoloInputNames.push_back(s.c_str());
        for (auto& s : m_yoloOutputNamesStorage) m_yoloOutputNames.push_back(s.c_str());

        spdlog::info("YOLO loaded: {} inputs, {} outputs",
                     m_yoloInputNames.size(), m_yoloOutputNames.size());

        // Load ResNet-18 model
        spdlog::info("Loading ResNet-18 model: {}", resnetPath);
        m_resnetSession = std::make_unique<Ort::Session>(
            m_env, resnetPath.c_str(), m_sessionOpts);

        for (size_t i = 0; i < m_resnetSession->GetInputCount(); i++) {
            auto name = m_resnetSession->GetInputNameAllocated(i, alloc);
            m_resnetInputNamesStorage.push_back(name.get());
        }
        for (size_t i = 0; i < m_resnetSession->GetOutputCount(); i++) {
            auto name = m_resnetSession->GetOutputNameAllocated(i, alloc);
            m_resnetOutputNamesStorage.push_back(name.get());
        }
        for (auto& s : m_resnetInputNamesStorage)  m_resnetInputNames.push_back(s.c_str());
        for (auto& s : m_resnetOutputNamesStorage) m_resnetOutputNames.push_back(s.c_str());

        spdlog::info("ResNet-18 loaded: {} inputs, {} outputs",
                     m_resnetInputNames.size(), m_resnetOutputNames.size());

        m_loaded = true;
        spdlog::info("Hazmat detector initialized successfully (CUDA={})", useCUDA);
        return true;

    } catch (const Ort::Exception& e) {
        spdlog::error("ONNX Runtime error: {}", e.what());
        m_loaded = false;
        return false;
    }
}

std::vector<HazmatDetection> HazmatDetector::detect(
    const cv::Mat& bgrFrame, int interval,
    float confThreshold, float nmsThreshold)
{
    if (!m_loaded || bgrFrame.empty()) return {};

    m_frameCount++;
    if (m_frameCount % interval != 0) return m_cached;

    auto detections = runYOLO(bgrFrame, confThreshold, nmsThreshold);

    // Stage 2: classify each detection crop with ResNet-18
    for (auto& det : detections) {
        // Clamp box to frame bounds
        int x1 = std::max(0, det.x);
        int y1 = std::max(0, det.y);
        int x2 = std::min(bgrFrame.cols, det.x + det.w);
        int y2 = std::min(bgrFrame.rows, det.y + det.h);
        if (x2 <= x1 || y2 <= y1) continue;

        cv::Mat crop = bgrFrame(cv::Rect(x1, y1, x2 - x1, y2 - y1));
        classifyCrop(crop, det);
    }

    m_cached = detections;
    return m_cached;
}

cv::Mat HazmatDetector::preprocessYOLO(const cv::Mat& bgrFrame,
                                        float& scaleX, float& scaleY,
                                        int& padLeft, int& padTop) {
    int origW = bgrFrame.cols;
    int origH = bgrFrame.rows;

    // Letterbox resize to YOLO_INPUT_SIZE x YOLO_INPUT_SIZE
    float scale = std::min(
        static_cast<float>(YOLO_INPUT_SIZE) / origW,
        static_cast<float>(YOLO_INPUT_SIZE) / origH);

    int newW = static_cast<int>(origW * scale);
    int newH = static_cast<int>(origH * scale);

    cv::Mat resized;
    cv::resize(bgrFrame, resized, cv::Size(newW, newH));

    // Create padded image (gray padding)
    cv::Mat padded(YOLO_INPUT_SIZE, YOLO_INPUT_SIZE, CV_8UC3,
                   cv::Scalar(114, 114, 114));
    padLeft = (YOLO_INPUT_SIZE - newW) / 2;
    padTop  = (YOLO_INPUT_SIZE - newH) / 2;
    resized.copyTo(padded(cv::Rect(padLeft, padTop, newW, newH)));

    scaleX = scale;
    scaleY = scale;
    return padded;
}

std::vector<HazmatDetection> HazmatDetector::runYOLO(
    const cv::Mat& bgrFrame,
    float confThreshold,
    float nmsThreshold)
{
    float scaleX, scaleY;
    int padLeft, padTop;
    cv::Mat padded = preprocessYOLO(bgrFrame, scaleX, scaleY, padLeft, padTop);

    // Convert to RGB float32, normalize to [0, 1], and layout NCHW
    cv::Mat rgb;
    cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0f / 255.0f);

    // HWC to CHW
    std::vector<float> inputTensor(3 * YOLO_INPUT_SIZE * YOLO_INPUT_SIZE);
    const int planeSize = YOLO_INPUT_SIZE * YOLO_INPUT_SIZE;
    for (int y = 0; y < YOLO_INPUT_SIZE; y++) {
        for (int x = 0; x < YOLO_INPUT_SIZE; x++) {
            const auto& px = rgb.at<cv::Vec3f>(y, x);
            inputTensor[0 * planeSize + y * YOLO_INPUT_SIZE + x] = px[0];
            inputTensor[1 * planeSize + y * YOLO_INPUT_SIZE + x] = px[1];
            inputTensor[2 * planeSize + y * YOLO_INPUT_SIZE + x] = px[2];
        }
    }

    // Create input tensor
    std::array<int64_t, 4> inputShape = {1, 3, YOLO_INPUT_SIZE, YOLO_INPUT_SIZE};
    auto memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value inputOrt = Ort::Value::CreateTensor<float>(
        memInfo, inputTensor.data(), inputTensor.size(),
        inputShape.data(), inputShape.size());

    // Run inference
    auto output = m_yoloSession->Run(
        Ort::RunOptions{nullptr},
        m_yoloInputNames.data(), &inputOrt, 1,
        m_yoloOutputNames.data(), m_yoloOutputNames.size());

    // Parse YOLOv8 output: shape [1, 14, 8400]
    // 14 = 4 (cx, cy, w, h) + 10 (class scores)
    auto* rawData = output[0].GetTensorMutableData<float>();
    auto shape = output[0].GetTensorTypeAndShapeInfo().GetShape();
    // shape = [1, numFeatures, numAnchors]
    int numFeatures = static_cast<int>(shape[1]);  // 14
    int numAnchors  = static_cast<int>(shape[2]);   // 8400
    int numClasses  = numFeatures - 4;              // 10

    std::vector<HazmatDetection> dets;

    for (int a = 0; a < numAnchors; a++) {
        // Find best class score for this anchor
        float maxScore = 0.0f;
        int bestClass = 0;
        for (int c = 0; c < numClasses; c++) {
            float score = rawData[(4 + c) * numAnchors + a];
            if (score > maxScore) {
                maxScore = score;
                bestClass = c;
            }
        }
        if (maxScore < confThreshold) continue;

        // Extract center-x, center-y, width, height (in 640x640 letterboxed space)
        float cx = rawData[0 * numAnchors + a];
        float cy = rawData[1 * numAnchors + a];
        float w  = rawData[2 * numAnchors + a];
        float h  = rawData[3 * numAnchors + a];

        // Convert to x1, y1, x2, y2 in letterboxed space
        float x1 = cx - w / 2.0f;
        float y1 = cy - h / 2.0f;
        float x2 = cx + w / 2.0f;
        float y2 = cy + h / 2.0f;

        // Remove padding and rescale to original image coordinates
        x1 = (x1 - padLeft) / scaleX;
        y1 = (y1 - padTop)  / scaleY;
        x2 = (x2 - padLeft) / scaleX;
        y2 = (y2 - padTop)  / scaleY;

        HazmatDetection det;
        det.x = static_cast<int>(x1);
        det.y = static_cast<int>(y1);
        det.w = static_cast<int>(x2 - x1);
        det.h = static_cast<int>(y2 - y1);
        det.yolo_confidence = maxScore;
        det.yolo_class = (bestClass >= 0 && bestClass < NUM_CLASSES)
                             ? CLASS_NAMES[bestClass]
                             : "unknown";
        dets.push_back(det);
    }

    nms(dets, nmsThreshold);
    return dets;
}

void HazmatDetector::classifyCrop(const cv::Mat& crop, HazmatDetection& det) {
    if (!m_resnetSession || crop.empty()) return;

    // Resize to 256, then center crop to 224 (matching torchvision transforms)
    cv::Mat resized;
    int shortSide = std::min(crop.cols, crop.rows);
    float resizeScale = 256.0f / shortSide;
    cv::resize(crop, resized,
               cv::Size(static_cast<int>(crop.cols * resizeScale),
                        static_cast<int>(crop.rows * resizeScale)));

    // Center crop 224x224
    int cx = resized.cols / 2;
    int cy = resized.rows / 2;
    int half = RESNET_INPUT_SIZE / 2;
    int x1 = std::max(0, cx - half);
    int y1 = std::max(0, cy - half);
    int cropW = std::min(RESNET_INPUT_SIZE, resized.cols - x1);
    int cropH = std::min(RESNET_INPUT_SIZE, resized.rows - y1);
    cv::Mat cropped = resized(cv::Rect(x1, y1, cropW, cropH));

    // Pad if needed (shouldn't happen normally)
    cv::Mat input224;
    if (cropW < RESNET_INPUT_SIZE || cropH < RESNET_INPUT_SIZE) {
        input224 = cv::Mat::zeros(RESNET_INPUT_SIZE, RESNET_INPUT_SIZE, CV_8UC3);
        cropped.copyTo(input224(cv::Rect(0, 0, cropW, cropH)));
    } else {
        input224 = cropped;
    }

    // Convert to RGB float, normalize with ImageNet stats
    cv::Mat rgb;
    cv::cvtColor(input224, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0f / 255.0f);

    // NCHW layout with ImageNet normalization
    const int planeSize = RESNET_INPUT_SIZE * RESNET_INPUT_SIZE;
    std::vector<float> inputTensor(3 * planeSize);
    for (int y = 0; y < RESNET_INPUT_SIZE; y++) {
        for (int x = 0; x < RESNET_INPUT_SIZE; x++) {
            const auto& px = rgb.at<cv::Vec3f>(y, x);
            inputTensor[0 * planeSize + y * RESNET_INPUT_SIZE + x] = (px[0] - MEAN[0]) / STD[0];
            inputTensor[1 * planeSize + y * RESNET_INPUT_SIZE + x] = (px[1] - MEAN[1]) / STD[1];
            inputTensor[2 * planeSize + y * RESNET_INPUT_SIZE + x] = (px[2] - MEAN[2]) / STD[2];
        }
    }

    std::array<int64_t, 4> inputShape = {1, 3, RESNET_INPUT_SIZE, RESNET_INPUT_SIZE};
    auto memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value inputOrt = Ort::Value::CreateTensor<float>(
        memInfo, inputTensor.data(), inputTensor.size(),
        inputShape.data(), inputShape.size());

    auto output = m_resnetSession->Run(
        Ort::RunOptions{nullptr},
        m_resnetInputNames.data(), &inputOrt, 1,
        m_resnetOutputNames.data(), m_resnetOutputNames.size());

    // Softmax + argmax
    auto* logits = output[0].GetTensorMutableData<float>();
    auto shape = output[0].GetTensorTypeAndShapeInfo().GetShape();
    int numCls = static_cast<int>(shape[1]);

    // Compute softmax
    float maxLogit = *std::max_element(logits, logits + numCls);
    float sumExp = 0.0f;
    std::vector<float> probs(numCls);
    for (int i = 0; i < numCls; i++) {
        probs[i] = std::exp(logits[i] - maxLogit);
        sumExp += probs[i];
    }
    int bestIdx = 0;
    float bestProb = 0.0f;
    for (int i = 0; i < numCls; i++) {
        probs[i] /= sumExp;
        if (probs[i] > bestProb) {
            bestProb = probs[i];
            bestIdx = i;
        }
    }

    det.resnet_class = (bestIdx >= 0 && bestIdx < NUM_CLASSES)
                           ? CLASS_NAMES[bestIdx]
                           : "unknown";
    det.resnet_confidence = bestProb;
}

void HazmatDetector::nms(std::vector<HazmatDetection>& dets, float nmsThresh) {
    if (dets.empty()) return;

    // Sort by YOLO confidence descending
    std::sort(dets.begin(), dets.end(),
              [](const HazmatDetection& a, const HazmatDetection& b) {
                  return a.yolo_confidence > b.yolo_confidence;
              });

    auto iou = [](const HazmatDetection& a, const HazmatDetection& b) -> float {
        int x1 = std::max(a.x, b.x);
        int y1 = std::max(a.y, b.y);
        int x2 = std::min(a.x + a.w, b.x + b.w);
        int y2 = std::min(a.y + a.h, b.y + b.h);
        if (x2 <= x1 || y2 <= y1) return 0.0f;
        float inter = static_cast<float>((x2 - x1) * (y2 - y1));
        float areaA = static_cast<float>(a.w * a.h);
        float areaB = static_cast<float>(b.w * b.h);
        return inter / (areaA + areaB - inter);
    };

    std::vector<bool> suppressed(dets.size(), false);
    std::vector<HazmatDetection> result;
    for (size_t i = 0; i < dets.size(); i++) {
        if (suppressed[i]) continue;
        result.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); j++) {
            if (!suppressed[j] && iou(dets[i], dets[j]) > nmsThresh) {
                suppressed[j] = true;
            }
        }
    }
    dets = std::move(result);
}

} // namespace detections
