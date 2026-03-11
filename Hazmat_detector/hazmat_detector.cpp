/**
 * Detector de materiales peligrosos con pipeline de dos etapas (C++)
 *   1. YOLO (hazmat_best.onnx)    – Detección y localización de señales hazmat
 *   2. ResNet-18 (best_resnet18.onnx) – Clasificación del tipo de material peligroso
 *
 * Requiere: OpenCV 4.x con módulo dnn
 * Los modelos .pt/.pth deben exportarse a ONNX antes de usar este programa.
 * Ver instrucciones de exportación al final del archivo o en el README.
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// ─── Constantes ────────────────────────────────────────────────────────────────

static const std::vector<std::string> RESNET_CLASS_NAMES = {
    "Combustible", "Corrosive", "Explosives", "Flammable",
    "Flammable_gas", "Fuel", "Oxidizer", "Peroxide",
    "Poison", "Radioactive"
};

static const float IMAGENET_MEAN[] = {0.485f, 0.456f, 0.406f};
static const float IMAGENET_STD[]  = {0.229f, 0.224f, 0.225f};

// ─── Estructuras ───────────────────────────────────────────────────────────────

struct Detection {
    int classId;
    std::string className;
    float confidence;
    cv::Rect box;
};

struct Classification {
    std::string className;
    float confidence;
};

// ─── Clase HazmatDetector ──────────────────────────────────────────────────────

class HazmatDetector {
public:
    HazmatDetector(const std::string& yoloPath, const std::string& resnetPath,
                   float confThreshold = 0.5f, float nmsThreshold = 0.45f)
        : confThreshold_(confThreshold), nmsThreshold_(nmsThreshold),
          hasClassifier_(false)
    {
        // --- Cargar modelo YOLO ---
        std::cout << "Cargando modelo YOLO: " << yoloPath << std::endl;
        yoloNet_ = cv::dnn::readNetFromONNX(yoloPath);
        if (yoloNet_.empty()) {
            std::cerr << "Error: No se pudo cargar el modelo YOLO: " << yoloPath << std::endl;
            return;
        }
        // Usar CUDA si está disponible, si no CPU
        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            yoloNet_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            yoloNet_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
            std::cout << "YOLO usando backend CUDA" << std::endl;
        } else {
            yoloNet_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
            yoloNet_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
            std::cout << "YOLO usando backend CPU" << std::endl;
        }
        std::cout << "Modelo YOLO cargado exitosamente" << std::endl;

        // --- Cargar modelo ResNet-18 (opcional) ---
        std::ifstream resnetFile(resnetPath);
        if (resnetFile.good()) {
            resnetFile.close();
            std::cout << "Cargando clasificador ResNet-18: " << resnetPath << std::endl;
            resnetNet_ = cv::dnn::readNetFromONNX(resnetPath);
            if (!resnetNet_.empty()) {
                if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
                    resnetNet_.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
                    resnetNet_.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
                } else {
                    resnetNet_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
                    resnetNet_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
                }
                hasClassifier_ = true;
                std::cout << "Clasificador ResNet-18 cargado exitosamente" << std::endl;
                std::cout << "Clases clasificables: ";
                for (const auto& name : RESNET_CLASS_NAMES)
                    std::cout << name << " ";
                std::cout << std::endl;
            } else {
                std::cerr << "Advertencia: No se pudo cargar ResNet-18" << std::endl;
            }
        } else {
            std::cout << "Clasificador " << resnetPath
                      << " no encontrado - solo se usara YOLO" << std::endl;
        }
    }

    bool isLoaded() const { return !yoloNet_.empty(); }

    // ─── Detección en imagen ───────────────────────────────────────────────────
    std::vector<Detection> detectImage(const std::string& imagePath, bool saveResult = true) {
        cv::Mat img = cv::imread(imagePath);
        if (img.empty()) {
            std::cerr << "Error: Imagen no encontrada: " << imagePath << std::endl;
            return {};
        }

        std::cout << "Analizando imagen: " << imagePath << std::endl;
        auto detections = runYolo(img);

        // Clasificar cada detección con ResNet-18
        for (auto& det : detections) {
            auto cls = classifyCrop(img, det.box);
            if (!cls.className.empty()) {
                std::cout << "  " << (det.classId + 1) << ". YOLO: " << det.className
                          << " (" << det.confidence * 100 << "%)" << std::endl;
                std::cout << "     ResNet-18: " << cls.className
                          << " (" << cls.confidence * 100 << "%)" << std::endl;
            }
        }

        if (detections.empty()) {
            std::cout << "No se detectaron materiales peligrosos" << std::endl;
        }

        if (saveResult) {
            cv::Mat annotated = drawDetections(img, detections);
            std::string status = detections.empty() ? "_SAFE" : "_HAZMAT_DETECTED";
            std::string outPath = insertBeforeExt(imagePath, status);
            cv::imwrite(outPath, annotated);
            std::cout << "Resultado guardado: " << outPath << std::endl;
        }

        return detections;
    }

    // ─── Detección en video ────────────────────────────────────────────────────
    void detectVideo(const std::string& videoPath, bool saveResult = true) {
        cv::VideoCapture cap(videoPath);
        if (!cap.isOpened()) {
            std::cerr << "Error: No se pudo abrir el video: " << videoPath << std::endl;
            return;
        }

        int fps    = static_cast<int>(cap.get(cv::CAP_PROP_FPS));
        int width  = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
        int total  = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT));

        cv::VideoWriter writer;
        std::string outPath;
        if (saveResult) {
            outPath = insertBeforeExt(videoPath, "_hazmat_analysis");
            writer.open(outPath, cv::VideoWriter::fourcc('m','p','4','v'),
                        fps, cv::Size(width, height));
        }

        std::cout << "Procesando video: " << videoPath << std::endl;
        int frameCount = 0;
        int framesWithHazmat = 0;
        cv::Mat frame;

        while (cap.read(frame)) {
            frameCount++;
            std::cout << "\rProcesando frame " << frameCount << "/" << total << std::flush;

            auto detections = runYolo(frame);
            cv::Mat annotated = drawDetections(frame, detections);

            if (!detections.empty()) framesWithHazmat++;
            if (saveResult) writer.write(annotated);
        }

        cap.release();
        if (saveResult) {
            writer.release();
            std::cout << "\nVideo analizado guardado: " << outPath << std::endl;
        }

        std::cout << "\n--- RESUMEN ---" << std::endl;
        std::cout << "  Frames totales: " << total << std::endl;
        std::cout << "  Frames con materiales peligrosos: " << framesWithHazmat << std::endl;
    }

    // ─── Listar cámaras ───────────────────────────────────────────────────────
    static std::vector<std::pair<int, cv::Size>> listCameras(int maxIndex = 5) {
        std::vector<std::pair<int, cv::Size>> cameras;
        for (int i = 0; i < maxIndex; i++) {
            cv::VideoCapture cap(i);
            if (cap.isOpened()) {
                int w = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
                int h = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
                cameras.push_back({i, cv::Size(w, h)});
                cap.release();
            }
        }
        return cameras;
    }

    // ─── Detección en webcam ───────────────────────────────────────────────────
    void detectWebcam(int cameraIndex = -1) {
        if (cameraIndex < 0) {
            auto cams = listCameras();
            if (cams.empty()) {
                std::cerr << "No se encontraron camaras" << std::endl;
                return;
            }
            std::cout << "Camaras disponibles:" << std::endl;
            for (const auto& [idx, sz] : cams)
                std::cout << "  " << idx << ": " << sz.width << "x" << sz.height << std::endl;

            std::cout << "Selecciona el indice de camara: ";
            std::cin >> cameraIndex;
            if (std::cin.fail()) { std::cin.clear(); cameraIndex = 0; }
        }

        std::cout << "Usando camara " << cameraIndex << std::endl;
        std::cout << "Presiona 'q' para salir" << std::endl;

        cv::VideoCapture cap(cameraIndex);
        if (!cap.isOpened()) {
            std::cerr << "Error: No se pudo abrir la camara " << cameraIndex << std::endl;
            return;
        }

        cv::Mat frame;
        while (cap.read(frame)) {
            auto detections = runYolo(frame);
            cv::Mat annotated = drawDetections(frame, detections);
            cv::imshow("HAZMAT DETECTOR - Deteccion en Tiempo Real", annotated);
            if (cv::waitKey(1) == 'q') break;
        }

        cap.release();
        cv::destroyAllWindows();
    }

private:
    cv::dnn::Net yoloNet_;
    cv::dnn::Net resnetNet_;
    float confThreshold_;
    float nmsThreshold_;
    bool hasClassifier_;

    // ─── Inferencia YOLO (YOLOv8 ONNX) ────────────────────────────────────────
    std::vector<Detection> runYolo(const cv::Mat& img) {
        const int inputSize = 640;

        // Letterbox: escalar manteniendo aspecto y padding gris
        float scale = std::min(static_cast<float>(inputSize) / img.cols,
                               static_cast<float>(inputSize) / img.rows);
        int newW = static_cast<int>(img.cols * scale);
        int newH = static_cast<int>(img.rows * scale);
        int dx = (inputSize - newW) / 2;
        int dy = (inputSize - newH) / 2;

        cv::Mat resized;
        cv::resize(img, resized, cv::Size(newW, newH));
        cv::Mat padded(inputSize, inputSize, CV_8UC3, cv::Scalar(114, 114, 114));
        resized.copyTo(padded(cv::Rect(dx, dy, newW, newH)));

        // Blob: 1/255, sin mean swap, RGB
        cv::Mat blob = cv::dnn::blobFromImage(padded, 1.0 / 255.0,
                                               cv::Size(inputSize, inputSize),
                                               cv::Scalar(), true, false);
        yoloNet_.setInput(blob);
        std::vector<cv::Mat> outputs;
        yoloNet_.forward(outputs, yoloNet_.getUnconnectedOutLayersNames());

        // YOLOv8 salida: [1, numClasses+4, numDetections]
        cv::Mat& out = outputs[0];
        // Transponer a [numDetections, numClasses+4]
        int rows = out.size[2]; // numDetections
        int cols = out.size[1]; // 4 + numClasses
        int numClasses = cols - 4;

        cv::Mat outT;
        // Reshape a 2D y transponer
        cv::Mat out2D = out.reshape(1, cols); // [cols, rows]
        cv::transpose(out2D, outT);           // [rows, cols]

        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        for (int i = 0; i < rows; i++) {
            const float* row = outT.ptr<float>(i);
            float cx = row[0];
            float cy = row[1];
            float w  = row[2];
            float h  = row[3];

            // Encontrar clase con mayor score
            const float* classScores = row + 4;
            int bestClassId = 0;
            float bestScore = classScores[0];
            for (int c = 1; c < numClasses; c++) {
                if (classScores[c] > bestScore) {
                    bestScore = classScores[c];
                    bestClassId = c;
                }
            }

            if (bestScore < confThreshold_) continue;

            // Convertir de coords en imagen padded a imagen original
            float x1 = (cx - w / 2.0f - dx) / scale;
            float y1 = (cy - h / 2.0f - dy) / scale;
            float bw  = w / scale;
            float bh  = h / scale;

            // Clamp a los bordes de la imagen
            x1 = std::max(0.0f, std::min(x1, static_cast<float>(img.cols)));
            y1 = std::max(0.0f, std::min(y1, static_cast<float>(img.rows)));
            bw = std::min(bw, static_cast<float>(img.cols) - x1);
            bh = std::min(bh, static_cast<float>(img.rows) - y1);

            boxes.emplace_back(static_cast<int>(x1), static_cast<int>(y1),
                               static_cast<int>(bw), static_cast<int>(bh));
            classIds.push_back(bestClassId);
            confidences.push_back(bestScore);
        }

        // NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confThreshold_, nmsThreshold_, indices);

        std::vector<Detection> detections;
        for (int idx : indices) {
            Detection det;
            det.classId    = classIds[idx];
            det.className  = "hazmat";  // Clase genérica YOLO
            det.confidence = confidences[idx];
            det.box        = boxes[idx];
            detections.push_back(det);
        }

        return detections;
    }

    // ─── Clasificación ResNet-18 ───────────────────────────────────────────────
    Classification classifyCrop(const cv::Mat& img, const cv::Rect& box) {
        Classification cls;
        cls.confidence = 0.0f;
        if (!hasClassifier_) return cls;

        // Recortar la región (clamped)
        cv::Rect safeBox = box & cv::Rect(0, 0, img.cols, img.rows);
        if (safeBox.width <= 0 || safeBox.height <= 0) return cls;
        cv::Mat crop = img(safeBox);

        // Preprocesamiento: resize a 256, center crop 224, normalización ImageNet
        cv::Mat resized;
        cv::resize(crop, resized, cv::Size(256, 256));

        // Center crop 224x224
        int offset = (256 - 224) / 2;
        cv::Mat cropped = resized(cv::Rect(offset, offset, 224, 224));

        // blobFromImage normaliza a [0,1] con scalefactor 1/255
        cv::Mat blob = cv::dnn::blobFromImage(cropped, 1.0 / 255.0,
                                               cv::Size(224, 224),
                                               cv::Scalar(), true, false);

        // Aplicar normalización ImageNet manualmente: (x - mean) / std
        // blob tiene forma [1, 3, 224, 224]
        float* blobData = blob.ptr<float>();
        int channelSize = 224 * 224;
        for (int c = 0; c < 3; c++) {
            for (int i = 0; i < channelSize; i++) {
                blobData[c * channelSize + i] =
                    (blobData[c * channelSize + i] - IMAGENET_MEAN[c]) / IMAGENET_STD[c];
            }
        }

        resnetNet_.setInput(blob);
        cv::Mat output = resnetNet_.forward();

        // Softmax y argmax
        float* data = output.ptr<float>();
        int numClasses = static_cast<int>(RESNET_CLASS_NAMES.size());

        // Softmax estable
        float maxVal = *std::max_element(data, data + numClasses);
        float sumExp = 0.0f;
        std::vector<float> probs(numClasses);
        for (int i = 0; i < numClasses; i++) {
            probs[i] = std::exp(data[i] - maxVal);
            sumExp += probs[i];
        }
        for (int i = 0; i < numClasses; i++)
            probs[i] /= sumExp;

        int bestIdx = static_cast<int>(
            std::max_element(probs.begin(), probs.end()) - probs.begin());

        cls.className  = RESNET_CLASS_NAMES[bestIdx];
        cls.confidence = probs[bestIdx];
        return cls;
    }

    // ─── Dibujar detecciones sobre imagen ──────────────────────────────────────
    cv::Mat drawDetections(const cv::Mat& img, const std::vector<Detection>& detections) {
        cv::Mat annotated = img.clone();

        for (const auto& det : detections) {
            // Bounding box verde
            cv::rectangle(annotated, det.box, cv::Scalar(0, 255, 0), 2);

            // Label YOLO arriba de la caja
            std::string yoloLabel = det.className + " " +
                std::to_string(static_cast<int>(det.confidence * 100)) + "%";
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(yoloLabel, cv::FONT_HERSHEY_SIMPLEX,
                                                 0.6, 2, &baseline);
            cv::Point textOrg(det.box.x, det.box.y - 5);
            if (textOrg.y < textSize.height) textOrg.y = det.box.y + textSize.height + 5;
            cv::putText(annotated, yoloLabel, textOrg,
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

            // Label ResNet-18 debajo de la caja (amarillo)
            auto cls = classifyCrop(img, det.box);
            if (!cls.className.empty() && cls.confidence > 0.0f) {
                std::string resnetLabel = cls.className + " " +
                    std::to_string(static_cast<int>(cls.confidence * 100)) + "%";
                cv::putText(annotated, resnetLabel,
                            cv::Point(det.box.x, det.box.y + det.box.height + 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            }
        }

        return annotated;
    }

    // ─── Utilidad: insertar texto antes de la extensión ────────────────────────
    static std::string insertBeforeExt(const std::string& path, const std::string& suffix) {
        size_t dot = path.rfind('.');
        if (dot == std::string::npos) return path + suffix;
        return path.substr(0, dot) + suffix + path.substr(dot);
    }
};

// ─── Menú principal ────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::string yoloPath   = "hazmat_best.onnx";
    std::string resnetPath = "best_resnet18.onnx";

    if (argc >= 2) yoloPath   = argv[1];
    if (argc >= 3) resnetPath = argv[2];

    std::cout << "=== DETECTOR DE MATERIALES PELIGROSOS (C++) ===" << std::endl;

    HazmatDetector detector(yoloPath, resnetPath);
    if (!detector.isLoaded()) {
        std::cerr << "Error al cargar los modelos. Saliendo." << std::endl;
        return 1;
    }

    while (true) {
        std::cout << "\n=============================================" << std::endl;
        std::cout << "Que tipo de analisis quieres hacer?" << std::endl;
        std::cout << "1. Analizar imagen" << std::endl;
        std::cout << "2. Analizar video" << std::endl;
        std::cout << "3. Deteccion en tiempo real (webcam)" << std::endl;
        std::cout << "4. Salir" << std::endl;

        std::string choice;
        std::cout << "Selecciona una opcion (1-4): ";
        std::getline(std::cin, choice);

        if (choice == "1") {
            std::string imgPath;
            std::cout << "Ruta de la imagen: ";
            std::getline(std::cin, imgPath);
            detector.detectImage(imgPath);

        } else if (choice == "2") {
            std::string vidPath;
            std::cout << "Ruta del video: ";
            std::getline(std::cin, vidPath);
            detector.detectVideo(vidPath);

        } else if (choice == "3") {
            detector.detectWebcam();

        } else if (choice == "4") {
            std::cout << "Mantente seguro!" << std::endl;
            break;
        } else {
            std::cout << "Opcion invalida" << std::endl;
        }
    }

    return 0;
}
