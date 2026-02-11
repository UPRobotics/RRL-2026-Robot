#include "camera_pkg/camera_manager.h"
#include "camera_pkg/settings_manager.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <numeric>
#include <cstdio>
#include <array>
#include <cstring>

// For Linux ping
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <netdb.h>

namespace camera_viewer {

// ── helpers to work with StreamVar ────────────────────────────────────
bool CameraManager::streamIsRunning(const StreamVar& sv) const {
    if (auto* cs = std::get_if<std::unique_ptr<CameraStream>>(&sv))
        return (*cs) && (*cs)->isRunning();
    if (auto* ts = std::get_if<std::unique_ptr<ThermalStream>>(&sv))
        return (*ts) && (*ts)->isRunning();
    return false;
}

void CameraManager::streamStop(StreamVar& sv) {
    if (auto* cs = std::get_if<std::unique_ptr<CameraStream>>(&sv)) {
        if (*cs) (*cs)->stop();
    } else if (auto* ts = std::get_if<std::unique_ptr<ThermalStream>>(&sv)) {
        if (*ts) (*ts)->stop();
    }
}

CameraStats CameraManager::streamGetStats(const StreamVar& sv) const {
    if (auto* cs = std::get_if<std::unique_ptr<CameraStream>>(&sv)) {
        if (*cs) return (*cs)->getStats();
    } else if (auto* ts = std::get_if<std::unique_ptr<ThermalStream>>(&sv)) {
        if (*ts) return (*ts)->getStats();
    }
    return CameraStats{};
}

SDL_Texture* CameraManager::streamGetTexture(StreamVar& sv) {
    if (auto* cs = std::get_if<std::unique_ptr<CameraStream>>(&sv)) {
        if (*cs) return (*cs)->getFrameTexture();
    } else if (auto* ts = std::get_if<std::unique_ptr<ThermalStream>>(&sv)) {
        if (*ts) return (*ts)->getFrameTexture();
    }
    return nullptr;
}

void CameraManager::streamUpdateTexture(StreamVar& sv) {
    if (auto* cs = std::get_if<std::unique_ptr<CameraStream>>(&sv)) {
        if (*cs && (*cs)->isRunning()) (*cs)->updateTextureFromMainThread();
    } else if (auto* ts = std::get_if<std::unique_ptr<ThermalStream>>(&sv)) {
        if (*ts && (*ts)->isRunning()) (*ts)->updateTextureFromMainThread();
    }
}

// ── CameraManager ────────────────────────────────────────────────────

CameraManager::CameraManager(SDL_Renderer* renderer, rclcpp::Node::SharedPtr node)
    : m_renderer(renderer)
    , m_rosNode(node)
    , m_lastRecoveryCheck(std::chrono::steady_clock::now())
{
    m_recoveryRunning = true;
    m_recoveryThread = std::thread(&CameraManager::recoveryThreadFunc, this);
}

CameraManager::~CameraManager() {
    m_recoveryRunning = false;
    if (m_recoveryThread.joinable()) m_recoveryThread.join();

    stopAll();

    if (m_discoveryThread.joinable()) {
        m_discoveryRunning = false;
        m_discoveryThread.join();
    }
}

void CameraManager::setCameraConfigs(const std::vector<CameraConfig>& configs) {
    std::lock_guard<std::mutex> lock(m_configMutex);
    m_configs = configs;

    {
        std::lock_guard<std::mutex> streamLock(m_streamsMutex);
        m_streams.clear();
    }

    spdlog::info("Configured {} cameras", configs.size());
}

void CameraManager::setStreamingSettings(const StreamingSettings& settings) {
    m_settings = settings;
    m_currentQuality = settings.default_quality;
}

int CameraManager::discoverCameras(DiscoveryCallback callback) {
    std::lock_guard<std::mutex> lock(m_configMutex);

    if (m_configs.empty()) {
        spdlog::warn("No cameras configured");
        return 0;
    }

    spdlog::info("Discovering cameras...");

    int availableCount = 0;
    int totalCount = static_cast<int>(m_configs.size());

    for (size_t i = 0; i < m_configs.size(); ++i) {
        auto& config = m_configs[i];

        if (!config.enabled) {
            spdlog::debug("Camera {} ({}) is disabled", i + 1, config.name);
            config.available = false;
            continue;
        }

        // Thermal cameras are always "available" (ROS topic)
        if (config.source_type == CameraSourceType::Thermal) {
            config.available = true;
            availableCount++;
            spdlog::info("Camera {} ({}) - THERMAL (topic: {})", i + 1, config.name, config.ros_topic);
            if (callback) callback(availableCount, totalCount);
            continue;
        }

        spdlog::debug("Pinging camera {} at {}", i + 1, config.ip);
        bool reachable = pingHost(config.ip, m_settings.ping_timeout_ms);
        config.available = reachable;

        if (reachable) {
            availableCount++;
            spdlog::info("Camera {} ({}) at {} - AVAILABLE", i + 1, config.name, config.ip);
        } else {
            spdlog::warn("Camera {} ({}) at {} - UNREACHABLE", i + 1, config.name, config.ip);
        }

        if (callback) callback(availableCount, totalCount);
    }

    spdlog::info("Discovery complete: {}/{} cameras available", availableCount, totalCount);
    return availableCount;
}

void CameraManager::startAll(StreamQuality quality) {
    std::lock_guard<std::mutex> configLock(m_configMutex);
    std::lock_guard<std::mutex> streamLock(m_streamsMutex);

    m_currentQuality = quality;
    m_streams.clear();
    m_streams.resize(m_configs.size());

    int startedCount = 0;
    for (size_t i = 0; i < m_configs.size(); ++i) {
        if (!m_configs[i].available || !m_configs[i].enabled) continue;

        if (m_configs[i].source_type == CameraSourceType::Thermal) {
            auto ts = std::make_unique<ThermalStream>(
                static_cast<int>(i), m_configs[i], m_renderer, m_rosNode);
            if (ts->start(quality)) startedCount++;
            m_streams[i] = std::move(ts);
        } else {
            auto cs = std::make_unique<CameraStream>(
                static_cast<int>(i), m_configs[i], m_renderer, m_decodeMode);
            if (cs->start(quality)) startedCount++;
            m_streams[i] = std::move(cs);
        }
    }

    spdlog::info("Started {} camera streams", startedCount);
}

void CameraManager::stopAll() {
    std::lock_guard<std::mutex> lock(m_streamsMutex);
    for (auto& sv : m_streams) streamStop(sv);
    m_streams.clear();
    spdlog::info("Stopped all camera streams");
}

void CameraManager::restartAll() {
    spdlog::info("Restarting all cameras");
    stopAll();
    discoverCameras();
    startAll(m_currentQuality);
}

bool CameraManager::startCamera(int index, StreamQuality quality) {
    std::lock_guard<std::mutex> configLock(m_configMutex);
    std::lock_guard<std::mutex> streamLock(m_streamsMutex);

    if (index < 0 || index >= static_cast<int>(m_configs.size())) {
        spdlog::error("Invalid camera index: {}", index);
        return false;
    }
    if (!m_configs[index].available) {
        spdlog::warn("Camera {} is not available", index + 1);
        return false;
    }

    if (m_streams.size() <= static_cast<size_t>(index))
        m_streams.resize(m_configs.size());

    streamStop(m_streams[index]);

    if (m_configs[index].source_type == CameraSourceType::Thermal) {
        auto ts = std::make_unique<ThermalStream>(index, m_configs[index], m_renderer, m_rosNode);
        bool ok = ts->start(quality);
        m_streams[index] = std::move(ts);
        return ok;
    } else {
        auto cs = std::make_unique<CameraStream>(index, m_configs[index], m_renderer, m_decodeMode);
        bool ok = cs->start(quality);
        m_streams[index] = std::move(cs);
        return ok;
    }
}

void CameraManager::stopCamera(int index) {
    std::lock_guard<std::mutex> lock(m_streamsMutex);
    if (index >= 0 && index < static_cast<int>(m_streams.size()))
        streamStop(m_streams[index]);
}

int CameraManager::getAvailableCameraCount() const {
    std::lock_guard<std::mutex> lock(m_configMutex);
    return static_cast<int>(std::count_if(m_configs.begin(), m_configs.end(),
        [](const CameraConfig& c) { return c.available && c.enabled; }));
}

std::vector<int> CameraManager::getAvailableCameraIndices() const {
    std::lock_guard<std::mutex> lock(m_configMutex);
    std::vector<int> indices;
    for (size_t i = 0; i < m_configs.size(); ++i) {
        if (m_configs[i].available && m_configs[i].enabled)
            indices.push_back(static_cast<int>(i));
    }
    return indices;
}

int CameraManager::getRunningCameraCount() const {
    std::lock_guard<std::mutex> lock(m_streamsMutex);
    return static_cast<int>(std::count_if(m_streams.begin(), m_streams.end(),
        [this](const StreamVar& s) { return streamIsRunning(s); }));
}

const CameraConfig& CameraManager::getCameraConfig(int index) const {
    std::lock_guard<std::mutex> lock(m_configMutex);
    static CameraConfig empty;
    if (index < 0 || index >= static_cast<int>(m_configs.size())) return empty;
    return m_configs[index];
}

int CameraManager::getCameraRotation(int index) const {
    std::lock_guard<std::mutex> lock(m_configMutex);
    if (index < 0 || index >= static_cast<int>(m_configs.size())) return 0;
    return m_configs[index].rotation_deg;
}

void CameraManager::setCameraRotation(int index, int rotationDeg) {
    {
        std::lock_guard<std::mutex> lock(m_configMutex);
        if (index < 0 || index >= static_cast<int>(m_configs.size())) return;
        rotationDeg %= 360;
        if (rotationDeg < 0) rotationDeg += 360;
        if (rotationDeg % 90 != 0) rotationDeg = 0;
        m_configs[index].rotation_deg = rotationDeg;
    }
    SettingsManager::instance().setCameraRotation(index, rotationDeg);
}

CameraStats CameraManager::getCameraStats(int index) const {
    std::lock_guard<std::mutex> lock(m_streamsMutex);
    if (index >= 0 && index < static_cast<int>(m_streams.size()))
        return streamGetStats(m_streams[index]);
    return CameraStats{};
}

SDL_Texture* CameraManager::getCameraTexture(int index) {
    std::lock_guard<std::mutex> lock(m_streamsMutex);
    if (index >= 0 && index < static_cast<int>(m_streams.size()))
        return streamGetTexture(m_streams[index]);
    return nullptr;
}

void CameraManager::updateTexturesFromMainThread() {
    std::lock_guard<std::mutex> lock(m_streamsMutex);
    for (auto& sv : m_streams) streamUpdateTexture(sv);
}

bool CameraManager::isCameraAvailable(int index) const {
    std::lock_guard<std::mutex> lock(m_configMutex);
    if (index < 0 || index >= static_cast<int>(m_configs.size())) return false;
    return m_configs[index].available && m_configs[index].enabled;
}

void CameraManager::setStateChangeCallback(StateChangeCallback callback) {
    m_stateChangeCallback = std::move(callback);
}

void CameraManager::setAllQuality(StreamQuality quality) {
    std::lock_guard<std::mutex> lock(m_streamsMutex);
    m_currentQuality = quality;
    for (auto& sv : m_streams) {
        if (auto* cs = std::get_if<std::unique_ptr<CameraStream>>(&sv)) {
            if (*cs) (*cs)->setQuality(quality);
        }
        // ThermalStream::setQuality is a no-op
    }
}

float CameraManager::getAverageFps() const {
    std::lock_guard<std::mutex> lock(m_streamsMutex);
    float total = 0.0f;
    int count = 0;
    for (const auto& sv : m_streams) {
        if (!streamIsRunning(sv)) continue;
        auto stats = streamGetStats(sv);
        if (stats.state == CameraState::Connected) {
            total += stats.current_fps;
            count++;
        }
    }
    return count > 0 ? total / count : 0.0f;
}

float CameraManager::getAverageLatency() const {
    std::lock_guard<std::mutex> lock(m_streamsMutex);
    float total = 0.0f;
    int count = 0;
    for (const auto& sv : m_streams) {
        if (!streamIsRunning(sv)) continue;
        auto stats = streamGetStats(sv);
        if (stats.state == CameraState::Connected) {
            total += stats.latency_ms;
            count++;
        }
    }
    return count > 0 ? total / count : 0.0f;
}

void CameraManager::checkAutoRecovery() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastRecoveryCheck);
    if (elapsed.count() < 1000) return;
    m_lastRecoveryCheck = now;

    std::lock_guard<std::mutex> lock(m_streamsMutex);

    for (size_t i = 0; i < m_streams.size(); ++i) {
        if (!streamIsRunning(m_streams[i])) continue;
        auto stats = streamGetStats(m_streams[i]);
        if (stats.state != CameraState::Connected) continue;

        auto timeSince = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - stats.last_frame_time);

        if (timeSince.count() > m_settings.frame_timeout_ms) {
            spdlog::warn("Camera {} frame timeout ({} ms) - queuing for restart...",
                         i + 1, timeSince.count());
            std::lock_guard<std::mutex> rl(m_recoveryMutex);
            if (std::find(m_recoveryQueue.begin(), m_recoveryQueue.end(), i) == m_recoveryQueue.end())
                m_recoveryQueue.push_back(i);
        }
    }
}

void CameraManager::recoveryThreadFunc() {
    while (m_recoveryRunning) {
        size_t idx = SIZE_MAX;
        {
            std::lock_guard<std::mutex> lock(m_recoveryMutex);
            if (!m_recoveryQueue.empty()) {
                idx = m_recoveryQueue.front();
                m_recoveryQueue.erase(m_recoveryQueue.begin());
            }
        }

        if (idx != SIZE_MAX) {
            spdlog::info("Camera {} - performing auto-restart...", idx + 1);
            {
                std::lock_guard<std::mutex> lock(m_streamsMutex);
                if (idx < m_streams.size()) streamStop(m_streams[idx]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            startCamera(static_cast<int>(idx), m_currentQuality);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

bool CameraManager::pingHost(const std::string& ip, int timeoutMs) {
    int timeoutSec = (timeoutMs + 999) / 1000;
    if (timeoutSec < 1) timeoutSec = 1;
    std::string cmd = "ping -c 1 -W " + std::to_string(timeoutSec) + " " + ip + " > /dev/null 2>&1";
    return (system(cmd.c_str()) == 0);
}

void CameraManager::discoveryThread(DiscoveryCallback callback) {
    m_discoveryRunning = true;
    int available = discoverCameras([this, &callback](int avail, int total) {
        if (callback && m_discoveryRunning) callback(avail, total);
    });
    m_discoveryRunning = false;
    if (callback) callback(available, static_cast<int>(m_configs.size()));
}

} // namespace camera_viewer
