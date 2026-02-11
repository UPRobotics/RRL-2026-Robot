#include "camera_pkg/thermal_stream.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace camera_viewer {

ThermalStream::ThermalStream(int cameraIndex, const CameraConfig& config,
                             SDL_Renderer* renderer, rclcpp::Node::SharedPtr node)
    : m_cameraIndex(cameraIndex)
    , m_config(config)
    , m_renderer(renderer)
    , m_node(node)
{
    m_stats.state = CameraState::Disconnected;
    m_lastFpsUpdate = std::chrono::steady_clock::now();
}

ThermalStream::~ThermalStream() {
    stop();

    std::lock_guard<std::mutex> lock(m_textureMutex);
    if (m_texture) {
        SDL_DestroyTexture(m_texture);
        m_texture = nullptr;
    }
}

bool ThermalStream::start(StreamQuality /*quality*/) {
    if (m_running.load()) {
        spdlog::warn("Thermal camera {} already running", m_cameraIndex + 1);
        return true;
    }

    if (!m_node) {
        spdlog::error("Thermal camera {} cannot start — no ROS2 node", m_cameraIndex + 1);
        return false;
    }

    const std::string& topic = m_config.ros_topic;
    if (topic.empty()) {
        spdlog::error("Thermal camera {} has no ROS topic configured", m_cameraIndex + 1);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats = CameraStats{};
        m_stats.state = CameraState::Connecting;
        m_stats.frame_width = DISPLAY_WIDTH;
        m_stats.frame_height = DISPLAY_HEIGHT;
        m_stats.last_frame_time = std::chrono::steady_clock::now();
    }

    // Create subscription with SensorDataQoS for best-effort, keep-last-1
    auto qos = rclcpp::SensorDataQoS();
    m_subscription = m_node->create_subscription<std_msgs::msg::Float32MultiArray>(
        topic, qos,
        std::bind(&ThermalStream::topicCallback, this, std::placeholders::_1));

    m_running = true;
    spdlog::info("Thermal camera {} subscribed to '{}'", m_cameraIndex + 1, topic);
    return true;
}

void ThermalStream::stop() {
    if (!m_running.load()) return;

    spdlog::info("Stopping thermal camera {}", m_cameraIndex + 1);
    m_running = false;
    m_subscription.reset();  // Unsubscribe

    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.state = CameraState::Disconnected;
    }
}

CameraState ThermalStream::getState() const {
    std::lock_guard<std::mutex> lock(m_statsMutex);
    return m_stats.state;
}

CameraStats ThermalStream::getStats() const {
    std::lock_guard<std::mutex> lock(m_statsMutex);
    return m_stats;
}

SDL_Texture* ThermalStream::getFrameTexture() {
    std::lock_guard<std::mutex> lock(m_textureMutex);
    return m_texture;
}

void ThermalStream::setFrameCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lock(m_callbackMutex);
    m_frameCallback = std::move(cb);
}

// ──────────────────────────────────────────────────────────────────────
//  ROS2 callback — runs on the executor thread, must be fast
// ──────────────────────────────────────────────────────────────────────
void ThermalStream::topicCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (!m_running.load()) return;

    const auto& data = msg->data;
    if (static_cast<int>(data.size()) != SENSOR_WIDTH * SENSOR_HEIGHT) {
        spdlog::warn("Thermal camera {} received {} floats, expected {}",
                     m_cameraIndex + 1, data.size(), SENSOR_WIDTH * SENSOR_HEIGHT);
        return;
    }

    // Compute min/max for dynamic range mapping
    float minT = *std::min_element(data.begin(), data.end());
    float maxT = *std::max_element(data.begin(), data.end());
    if (maxT - minT < 1.0f) {
        // Avoid degenerate range — center around mean with ±0.5 °C
        float mid = (minT + maxT) * 0.5f;
        minT = mid - 0.5f;
        maxT = mid + 0.5f;
    }

    // Build upscaled BGRA image (nearest-neighbor)
    const int pitch = DISPLAY_WIDTH * 4;
    std::vector<uint8_t> pixels(pitch * DISPLAY_HEIGHT);

    for (int sy = 0; sy < SENSOR_HEIGHT; ++sy) {
        for (int sx = 0; sx < SENSOR_WIDTH; ++sx) {
            float temp = data[sy * SENSOR_WIDTH + sx];
            uint8_t b, g, r, a;
            temperatureToColor(temp, minT, maxT, b, g, r, a);

            // Fill the DISPLAY_SCALE x DISPLAY_SCALE block
            for (int dy = 0; dy < DISPLAY_SCALE; ++dy) {
                int py = sy * DISPLAY_SCALE + dy;
                for (int dx = 0; dx < DISPLAY_SCALE; ++dx) {
                    int px = sx * DISPLAY_SCALE + dx;
                    int idx = (py * DISPLAY_WIDTH + px) * 4;
                    pixels[idx + 0] = b;
                    pixels[idx + 1] = g;
                    pixels[idx + 2] = r;
                    pixels[idx + 3] = a;
                }
            }
        }
    }

    // Store for main-thread upload
    {
        std::lock_guard<std::mutex> lock(m_pendingFrameMutex);
        m_pendingFrameData = std::move(pixels);
        m_pendingFrameWidth = DISPLAY_WIDTH;
        m_pendingFrameHeight = DISPLAY_HEIGHT;
        m_pendingFramePitch = pitch;
        m_hasPendingFrame.store(true);
    }

    // Update stats
    auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(m_statsMutex);
        m_stats.state = CameraState::Connected;
        m_stats.total_frames++;
        m_stats.last_frame_time = now;
        m_frameCountSinceLastUpdate++;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastFpsUpdate);
        if (elapsed.count() >= 1000) {
            m_stats.current_fps = m_frameCountSinceLastUpdate * 1000.0f / elapsed.count();
            m_frameCountSinceLastUpdate = 0;
            m_lastFpsUpdate = now;
        }
    }
}

// ──────────────────────────────────────────────────────────────────────
//  Main-thread texture upload (same pattern as CameraStream)
// ──────────────────────────────────────────────────────────────────────
bool ThermalStream::updateTextureFromMainThread() {
    if (!m_hasPendingFrame.load()) return false;

    std::lock_guard<std::mutex> frameLock(m_pendingFrameMutex);
    std::lock_guard<std::mutex> texLock(m_textureMutex);

    if (m_pendingFrameData.empty()) return false;

    // Create texture if needed
    if (!m_texture) {
        m_texture = SDL_CreateTexture(
            m_renderer,
            SDL_PIXELFORMAT_ARGB8888,
            SDL_TEXTUREACCESS_STREAMING,
            m_pendingFrameWidth, m_pendingFrameHeight);

        if (!m_texture) {
            spdlog::error("Thermal camera {} failed to create texture: {}",
                         m_cameraIndex + 1, SDL_GetError());
            return false;
        }
        spdlog::info("Thermal camera {} texture created ({}x{})",
                     m_cameraIndex + 1, m_pendingFrameWidth, m_pendingFrameHeight);
    }

    // Upload via SDL_LockTexture (zero-copy path)
    void* texPixels = nullptr;
    int texPitch = 0;
    if (SDL_LockTexture(m_texture, nullptr, &texPixels, &texPitch) == 0) {
        if (texPitch == m_pendingFramePitch) {
            std::memcpy(texPixels, m_pendingFrameData.data(),
                        m_pendingFramePitch * m_pendingFrameHeight);
        } else {
            const uint8_t* src = m_pendingFrameData.data();
            uint8_t* dst = static_cast<uint8_t*>(texPixels);
            int rowBytes = std::min(texPitch, m_pendingFramePitch);
            for (int row = 0; row < m_pendingFrameHeight; ++row) {
                std::memcpy(dst, src, rowBytes);
                src += m_pendingFramePitch;
                dst += texPitch;
            }
        }
        SDL_UnlockTexture(m_texture);
    } else {
        SDL_UpdateTexture(m_texture, nullptr,
                          m_pendingFrameData.data(), m_pendingFramePitch);
    }

    m_hasPendingFrame.store(false);
    return true;
}

// ──────────────────────────────────────────────────────────────────────
//  Iron-bow false-color palette  (cold → hot)
//    black → dark blue → purple → red → orange → yellow → white
// ──────────────────────────────────────────────────────────────────────
void ThermalStream::temperatureToColor(float temp, float minTemp, float maxTemp,
                                       uint8_t& b, uint8_t& g, uint8_t& r, uint8_t& a) {
    float t = (temp - minTemp) / (maxTemp - minTemp);
    t = std::clamp(t, 0.0f, 1.0f);
    a = 255;

    // 5-stop gradient
    if (t < 0.25f) {
        // black → dark blue
        float s = t / 0.25f;
        r = 0;
        g = 0;
        b = static_cast<uint8_t>(s * 180);
    } else if (t < 0.5f) {
        // dark blue → purple/magenta
        float s = (t - 0.25f) / 0.25f;
        r = static_cast<uint8_t>(s * 200);
        g = 0;
        b = static_cast<uint8_t>(180 + s * 40);
    } else if (t < 0.7f) {
        // purple → red-orange
        float s = (t - 0.5f) / 0.2f;
        r = static_cast<uint8_t>(200 + s * 55);
        g = static_cast<uint8_t>(s * 100);
        b = static_cast<uint8_t>(220 - s * 220);
    } else if (t < 0.9f) {
        // red-orange → yellow
        float s = (t - 0.7f) / 0.2f;
        r = 255;
        g = static_cast<uint8_t>(100 + s * 155);
        b = 0;
    } else {
        // yellow → white
        float s = (t - 0.9f) / 0.1f;
        r = 255;
        g = 255;
        b = static_cast<uint8_t>(s * 255);
    }
}

} // namespace camera_viewer
