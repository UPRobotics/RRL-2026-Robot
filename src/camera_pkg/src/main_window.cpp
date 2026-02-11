#include "camera_pkg/main_window.h"
#include "camera_pkg/console_window.h"
#include "camera_pkg/console_sink.h"
#include "camera_pkg/stats_panel.h"
#include "camera_pkg/camera_grid.h"
#include "camera_pkg/camera_manager.h"
#include "camera_pkg/ui_helpers.h"
#include "camera_pkg/settings_manager.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <regex>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <cstdio>

namespace camera_viewer {

MainWindow::MainWindow(const std::string& title, int width, int height, DecodeMode decodeMode)
    : m_title(title)
    , m_windowWidth(width)
    , m_windowHeight(height)
    , m_running(false)
    , m_isInitialized(false)
    , m_window(nullptr)
    , m_renderer(nullptr)
    , m_currentViewMode(ViewMode::GRID_NXN)
    , m_consolVisible(false)
    , m_isFullscreen(false)
    , m_activeCameraCount(4)
    , m_mainAreaY(TOOLBAR_HEIGHT)
    , m_mainAreaHeight(height - TOOLBAR_HEIGHT - STATSBAR_HEIGHT)
    , m_prevProcJiffies(0)
    , m_prevTotalJiffies(0)
    , m_hasPrevCpuSample(false)
    , m_decodeMode(decodeMode)
{
}

MainWindow::~MainWindow() {
    shutdown();
}

bool MainWindow::initialize() {
    if (m_isInitialized) {
        spdlog::warn("MainWindow already initialized");
        return true;
    }

    // Load settings
    SettingsManager::instance().load();

    // Initialize SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        spdlog::error("SDL initialization failed: {}", SDL_GetError());
        return false;
    }

    // Initialize font manager
    if (!FontManager::instance().initialize()) {
        spdlog::error("Font manager initialization failed");
        SDL_Quit();
        return false;
    }

    // Get window position and size from settings
    auto& settings = SettingsManager::instance();
    int windowX = SDL_WINDOWPOS_CENTERED;
    int windowY = SDL_WINDOWPOS_CENTERED;
    
    if (settings.shouldRememberWindowPosition() && settings.getLastWindowX() >= 0) {
        windowX = settings.getLastWindowX();
        windowY = settings.getLastWindowY();
    }
    
    if (settings.shouldRememberWindowSize()) {
        m_windowWidth = settings.getLastWindowWidth();
        m_windowHeight = settings.getLastWindowHeight();
    }

    // Create window
    m_window = SDL_CreateWindow(
        m_title.c_str(),
        windowX,
        windowY,
        m_windowWidth,
        m_windowHeight,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );

    if (!m_window) {
        spdlog::error("Window creation failed: {}", SDL_GetError());
        FontManager::instance().shutdown();
        SDL_Quit();
        return false;
    }

    // Create renderer
    // No VSync — eliminates up to 16.67ms latency per frame
    m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED);
    if (!m_renderer) {
        spdlog::error("Renderer creation failed: {}", SDL_GetError());
        SDL_DestroyWindow(m_window);
        SDL_Quit();
        return false;
    }

    // Initialize UI components
    m_consoleWindow = std::make_unique<ConsoleWindow>();
    m_statsPanel = std::make_unique<StatsPanel>();
    m_cameraGrid = std::make_unique<CameraGrid>();
    
    // Initialize camera manager (ROS node is set later from main.cpp)
    m_cameraManager = std::make_unique<CameraManager>(m_renderer);
    m_cameraManager->setDecodeMode(m_decodeMode);
    m_cameraManager->setCameraConfigs(settings.getCameraConfigs());
    m_cameraManager->setStreamingSettings(settings.getStreamingSettings());
    
    // Connect camera grid to camera manager
    m_cameraGrid->setCameraManager(m_cameraManager.get());
    m_cameraGrid->setViewMode(m_currentViewMode);
    
    // Discover cameras at startup (before main loop)
    spdlog::info("Discovering cameras at startup...");
    int available = m_cameraManager->discoverCameras([](int found, int total) {
        spdlog::debug("Discovery progress: {}/{}", found, total);
    });
    
    // Update camera grid with available indices
    auto availableIndices = m_cameraManager->getAvailableCameraIndices();
    m_cameraGrid->setAvailableCameraIndices(availableIndices);
    m_activeCameraCount = available;
    
    // Load persistent 2x2 slot assignments
    m_cameraGrid->set2x2SlotAssignments(settings.getGrid2x2Slots());
    
    if (!availableIndices.empty()) {
        int firstIdx = availableIndices.front();
        m_pingCameraIp = m_cameraManager->getCameraConfig(firstIdx).ip;
        if (!m_pingCameraIp.empty()) {
            spdlog::info("Telemetry ping target set to camera {} ({})", firstIdx + 1, m_pingCameraIp);
        }
    }
    
    spdlog::info("Found {} available cameras", available);

    // Set up logging to redirect all messages to console window
    auto console_sink = std::make_shared<ConsoleSink>(m_consoleWindow.get());
    auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    
    auto logger = std::make_shared<spdlog::logger>("multi_sink", 
        spdlog::sinks_init_list{console_sink, stdout_sink});
    logger->set_level(spdlog::level::trace); // Capture all log levels
    spdlog::set_default_logger(logger);

    // Setup toolbar buttons
    int buttonX = BUTTON_MARGIN;
    m_toolbarButtons.clear();

    // Start Cameras button
    ButtonRect startBtn;
    startBtn.rect = {buttonX, (TOOLBAR_HEIGHT - BUTTON_HEIGHT) / 2, BUTTON_WIDTH, BUTTON_HEIGHT};
    startBtn.label = "Start Cameras";
    startBtn.hovered = false;
    startBtn.enabled = true;
    m_toolbarButtons.push_back(startBtn);
    buttonX += BUTTON_WIDTH + BUTTON_MARGIN;

    // Stop Cameras button
    ButtonRect stopBtn;
    stopBtn.rect = {buttonX, (TOOLBAR_HEIGHT - BUTTON_HEIGHT) / 2, BUTTON_WIDTH, BUTTON_HEIGHT};
    stopBtn.label = "Stop Cameras";
    stopBtn.hovered = false;
    stopBtn.enabled = false; // Disabled initially
    m_toolbarButtons.push_back(stopBtn);
    buttonX += BUTTON_WIDTH + BUTTON_MARGIN;

    // Restart Cameras button
    ButtonRect restartBtn;
    restartBtn.rect = {buttonX, (TOOLBAR_HEIGHT - BUTTON_HEIGHT) / 2, BUTTON_WIDTH, BUTTON_HEIGHT};
    restartBtn.label = "Restart Cameras";
    restartBtn.hovered = false;
    restartBtn.enabled = false; // Disabled initially
    m_toolbarButtons.push_back(restartBtn);
    buttonX += BUTTON_WIDTH + BUTTON_MARGIN * 3;

    // Console button
    ButtonRect consoleBtn;
    consoleBtn.rect = {buttonX, (TOOLBAR_HEIGHT - BUTTON_HEIGHT) / 2, BUTTON_WIDTH, BUTTON_HEIGHT};
    consoleBtn.label = "Show Console";
    consoleBtn.hovered = false;
    consoleBtn.enabled = true;
    m_toolbarButtons.push_back(consoleBtn);

    m_isInitialized = true;
    m_running = true;
    startTelemetry();
    spdlog::info("MainWindow initialized successfully ({}x{})", m_windowWidth, m_windowHeight);
    return true;
}

bool MainWindow::spinOnce() {
    if (!m_isInitialized) {
        spdlog::error("Cannot spinOnce: MainWindow not initialized");
        return false;
    }

    Uint32 frameStart = SDL_GetTicks();

    handleEvents();
    render();

    // Cap at ~240 FPS (4ms) — minimises display lag for new frames
    Uint32 elapsed = SDL_GetTicks() - frameStart;
    if (elapsed < 4) {
        SDL_Delay(4 - elapsed);
    }

    return m_running;
}

void MainWindow::handleEvents() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        // Handle console window events if visible
        if (m_consolVisible && m_consoleWindow) {
            m_consoleWindow->handleEvent(event);
            
            // Sync console visibility state if it was closed
            if (!m_consoleWindow->isVisible() && m_consolVisible) {
                m_consolVisible = false;
                m_toolbarButtons[3].label = "Show Console";
            }
        }

        switch (event.type) {
            case SDL_QUIT:
                m_running = false;
                break;

            case SDL_WINDOWEVENT:
                if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
                    handleResize(event.window.data1, event.window.data2);
                } else if (event.window.event == SDL_WINDOWEVENT_CLOSE) {
                    // Handle main window close button
                    if (event.window.windowID == SDL_GetWindowID(m_window)) {
                        m_running = false;
                    }
                }
                break;

            case SDL_MOUSEBUTTONDOWN:
                if (event.button.button == SDL_BUTTON_LEFT) {
                    handleMouseClick(event.button.x, event.button.y);
                }
                break;

            case SDL_MOUSEMOTION: {
                // Update button hover states
                int mouseX = event.motion.x;
                int mouseY = event.motion.y;
                for (auto& button : m_toolbarButtons) {
                    button.hovered = UIHelpers::pointInRect(mouseX, mouseY, button.rect);
                }
                break;
            }

            case SDL_KEYDOWN:
                handleKeyPress(event.key.keysym.sym);
                break;
        }
    }
}

void MainWindow::handleMouseClick(int x, int y) {
    // Check toolbar button clicks
    for (size_t i = 0; i < m_toolbarButtons.size(); ++i) {
        if (m_toolbarButtons[i].enabled && UIHelpers::pointInRect(x, y, m_toolbarButtons[i].rect)) {
            switch (i) {
                case 0: onStartCamerasClicked(); break;
                case 1: onStopCamerasClicked(); break;
                case 2: onRestartCamerasClicked(); break;
                case 3: onToggleConsoleClicked(); break;
            }
            break;
        }
    }
}

void MainWindow::handleResize(int width, int height) {
    m_windowWidth = width;
    m_windowHeight = height;
    m_mainAreaHeight = height - TOOLBAR_HEIGHT - STATSBAR_HEIGHT;
    spdlog::debug("Window resized to {}x{}", width, height);
}

void MainWindow::handleKeyPress(SDL_Keycode key) {
    auto& settings = SettingsManager::instance();
    
    // Check keybindings
    if (settings.isKeyForAction(key, "quit") || settings.isKeyForAction(key, "quit_alt")) {
        m_running = false;
    }
    else if (settings.isKeyForAction(key, "toggle_fullscreen")) {
        toggleFullscreen();
    }
    else if (settings.isKeyForAction(key, "toggle_console")) {
        onToggleConsoleClicked();
    }
    else if (settings.isKeyForAction(key, "view_mode_fullscreen")) {
        onViewModeChanged(ViewMode::FULLSCREEN);
    }
    else if (settings.isKeyForAction(key, "view_mode_2x2")) {
        onViewModeChanged(ViewMode::GRID_2X2);
    }
    else if (settings.isKeyForAction(key, "view_mode_grid")) {
        onViewModeChanged(ViewMode::GRID_NXN);
    }
    else if (settings.isKeyForAction(key, "camera_previous")) {
        // Navigate to previous camera in fullscreen mode
        if (m_currentViewMode == ViewMode::FULLSCREEN && m_cameraGrid->getAvailableCameraCount() > 0) {
            int currentIndex = m_cameraGrid->getSelectedCameraIndex();
            int count = m_cameraGrid->getAvailableCameraCount();
            int newIndex = (currentIndex - 1 + count) % count;
            m_cameraGrid->setSelectedCameraIndex(newIndex);
            int realCamNum = m_cameraGrid->getSelectedRealCameraIndex() + 1;
            spdlog::info("Switched to Camera {}", realCamNum);
        }
    }
    else if (settings.isKeyForAction(key, "camera_next")) {
        // Navigate to next camera in fullscreen mode
        if (m_currentViewMode == ViewMode::FULLSCREEN && m_cameraGrid->getAvailableCameraCount() > 0) {
            int currentIndex = m_cameraGrid->getSelectedCameraIndex();
            int count = m_cameraGrid->getAvailableCameraCount();
            int newIndex = (currentIndex + 1) % count;
            m_cameraGrid->setSelectedCameraIndex(newIndex);
            int realCamNum = m_cameraGrid->getSelectedRealCameraIndex() + 1;
            spdlog::info("Switched to Camera {}", realCamNum);
        }
    }
    else if (key == SDLK_r) {
        // Rotate current camera 90 degrees clockwise (persistent) when in fullscreen
        if (m_currentViewMode == ViewMode::FULLSCREEN && m_cameraGrid && m_cameraGrid->getAvailableCameraCount() > 0) {
            int realIndex = m_cameraGrid->getSelectedRealCameraIndex();
            if (realIndex >= 0 && m_cameraManager) {
                int currentRotation = m_cameraManager->getCameraRotation(realIndex);
                int nextRotation = (currentRotation + 90) % 360;
                m_cameraManager->setCameraRotation(realIndex, nextRotation);
                spdlog::info("Camera {} rotation set to {} degrees", realIndex + 1, nextRotation);
            }
        }
    }
    // Numpad keys for 2x2 slot assignment
    // KP_7 = top-left (slot 0), KP_9 = top-right (slot 1)
    // KP_1 = bottom-left (slot 2), KP_3 = bottom-right (slot 3)
    else if (key == SDLK_KP_7 || key == SDLK_KP_9 || key == SDLK_KP_1 || key == SDLK_KP_3) {
        if (m_currentViewMode == ViewMode::GRID_2X2 && m_cameraGrid && m_cameraGrid->getAvailableCameraCount() > 0) {
            int slotIndex = -1;
            const char* slotName = "";
            if (key == SDLK_KP_7)      { slotIndex = 0; slotName = "top-left"; }
            else if (key == SDLK_KP_9) { slotIndex = 1; slotName = "top-right"; }
            else if (key == SDLK_KP_1) { slotIndex = 2; slotName = "bottom-left"; }
            else if (key == SDLK_KP_3) { slotIndex = 3; slotName = "bottom-right"; }
            
            m_cameraGrid->cycle2x2Slot(slotIndex);
            auto slots = m_cameraGrid->get2x2SlotAssignments();
            int camNum = slots[slotIndex] + 1;
            spdlog::info("2x2 {} slot set to Camera {}", slotName, camNum);
            
            // Persist
            SettingsManager::instance().setGrid2x2Slots(slots);
        }
    }
}

void MainWindow::render() {
    // Update camera textures from main thread (SDL requirement)
    if (m_cameraManager) {
        m_cameraManager->updateTexturesFromMainThread();
        m_cameraManager->checkAutoRecovery();
    }

    // Pull latest telemetry values (updated by background thread once per second)
    if (m_statsPanel) {
        m_statsPanel->updateCpuUsage(m_cpuUsageAtomic.load());
        m_statsPanel->updateRamUsage(m_ramUsageAtomic.load());
        m_statsPanel->updateLatency(m_latencyAtomic.load());
        m_statsPanel->updateGpuUsage(m_gpuUsageAtomic.load());
    }
    
    // Clear screen
    SDL_SetRenderDrawColor(m_renderer, 
        Colors::BACKGROUND.r, Colors::BACKGROUND.g, Colors::BACKGROUND.b, Colors::BACKGROUND.a);
    SDL_RenderClear(m_renderer);

    // Render UI components
    renderToolbar();
    renderMainArea();
    renderStatsBar();

    // Present
    SDL_RenderPresent(m_renderer);

    // Render console if visible
    if (m_consolVisible && m_consoleWindow) {
        m_consoleWindow->render();
    }
}

void MainWindow::renderToolbar() {
    // Toolbar background
    SDL_Rect toolbarRect = {0, 0, m_windowWidth, TOOLBAR_HEIGHT};
    SDL_SetRenderDrawColor(m_renderer, 
        Colors::TOOLBAR_BG.r, Colors::TOOLBAR_BG.g, Colors::TOOLBAR_BG.b, Colors::TOOLBAR_BG.a);
    SDL_RenderFillRect(m_renderer, &toolbarRect);

    // Toolbar border
    SDL_SetRenderDrawColor(m_renderer, 
        Colors::BORDER.r, Colors::BORDER.g, Colors::BORDER.b, Colors::BORDER.a);
    SDL_RenderDrawLine(m_renderer, 0, TOOLBAR_HEIGHT - 1, m_windowWidth, TOOLBAR_HEIGHT - 1);

    // Render buttons
    for (const auto& button : m_toolbarButtons) {
        UIHelpers::drawButton(m_renderer, button.rect, button.label, 
                             button.hovered, false, button.enabled);
    }
}

void MainWindow::renderMainArea() {
    // Main area background
    SDL_Rect mainAreaRect = {0, m_mainAreaY, m_windowWidth, m_mainAreaHeight};
    SDL_SetRenderDrawColor(m_renderer, 
        Colors::BACKGROUND.r, Colors::BACKGROUND.g, Colors::BACKGROUND.b, Colors::BACKGROUND.a);
    SDL_RenderFillRect(m_renderer, &mainAreaRect);

    // Render camera grid
    if (m_cameraGrid) {
        m_cameraGrid->render(m_renderer, 0, m_mainAreaY, m_windowWidth, m_mainAreaHeight);
    }
}

void MainWindow::renderStatsBar() {
    int statsY = m_windowHeight - STATSBAR_HEIGHT;
    
    // Stats bar background
    SDL_Rect statsRect = {0, statsY, m_windowWidth, STATSBAR_HEIGHT};
    SDL_SetRenderDrawColor(m_renderer, 
        Colors::STATSBAR_BG.r, Colors::STATSBAR_BG.g, Colors::STATSBAR_BG.b, Colors::STATSBAR_BG.a);
    SDL_RenderFillRect(m_renderer, &statsRect);

    // Stats bar border
    SDL_SetRenderDrawColor(m_renderer, 
        Colors::BORDER.r, Colors::BORDER.g, Colors::BORDER.b, Colors::BORDER.a);
    SDL_RenderDrawLine(m_renderer, 0, statsY, m_windowWidth, statsY);

    // Render stats
    if (m_statsPanel) {
        m_statsPanel->render(m_renderer, 0, statsY, m_windowWidth, STATSBAR_HEIGHT);
    }
}

void MainWindow::onStartCamerasClicked() {
    spdlog::info("Start Cameras clicked");
    
    if (m_activeCameraCount == 0) {
        spdlog::warn("No cameras available - run discovery first");
        return;
    }
    
    // Start all available camera streams
    m_cameraManager->startAll();
    
    m_toolbarButtons[0].enabled = false; // Disable Start
    m_toolbarButtons[1].enabled = true;  // Enable Stop
    m_toolbarButtons[2].enabled = true;  // Enable Restart
    
    spdlog::info("Started {} camera streams", m_activeCameraCount);
}

void MainWindow::onStopCamerasClicked() {
    spdlog::info("Stop Cameras clicked");
    
    // Stop all camera streams
    if (m_cameraManager) {
        m_cameraManager->stopAll();
    }
    
    // Keep available camera info but mark as stopped
    m_toolbarButtons[0].enabled = true;  // Enable Start
    m_toolbarButtons[1].enabled = false; // Disable Stop
    m_toolbarButtons[2].enabled = false; // Disable Restart
}

void MainWindow::onRestartCamerasClicked() {
    spdlog::info("Restart Cameras clicked");
    
    // Stop first
    if (m_cameraManager) {
        m_cameraManager->stopAll();
    }
    
    // Re-discover cameras
    spdlog::info("Re-discovering cameras...");
    int available = m_cameraManager->discoverCameras([](int found, int total) {
        spdlog::debug("Discovery progress: {}/{}", found, total);
    });
    
    // Update camera grid with available indices
    auto availableIndices = m_cameraManager->getAvailableCameraIndices();
    m_cameraGrid->setAvailableCameraIndices(availableIndices);
    m_activeCameraCount = available;
    
    if (available > 0) {
        m_cameraManager->startAll();
        m_toolbarButtons[0].enabled = false;
        m_toolbarButtons[1].enabled = true;
        m_toolbarButtons[2].enabled = true;
    } else {
        m_toolbarButtons[0].enabled = true;
        m_toolbarButtons[1].enabled = false;
        m_toolbarButtons[2].enabled = false;
    }
    
    spdlog::info("Restarted with {} cameras", available);
}

void MainWindow::onToggleConsoleClicked() {
    m_consolVisible = !m_consolVisible;
    
    if (m_consolVisible) {
        // Create or show console window
        if (!m_consoleWindow->isVisible()) {
            int consoleX = 100;
            int consoleY = 100;
            int consoleWidth = 800;
            int consoleHeight = 500;
            // create() will reuse existing window if already created
            m_consoleWindow->create(consoleX, consoleY, consoleWidth, consoleHeight);
        } else {
            // Just raise to front if already visible
            m_consoleWindow->setVisible(true);
        }
        m_toolbarButtons[3].label = "Hide Console";
        spdlog::info("Console window opened");
    } else {
        m_consoleWindow->setVisible(false);
        m_toolbarButtons[3].label = "Show Console";
        spdlog::info("Console window hidden");
    }
}

void MainWindow::onViewModeChanged(ViewMode mode) {
    m_currentViewMode = mode;
    m_cameraGrid->setViewMode(mode);
    
    const char* modeStr = "Unknown";
    switch (mode) {
        case ViewMode::FULLSCREEN: modeStr = "Fullscreen"; break;
        case ViewMode::GRID_2X2: modeStr = "2x2 Grid"; break;
        case ViewMode::GRID_NXN: modeStr = "NxN Grid"; break;
    }
    spdlog::info("View mode changed to: {}", modeStr);
}

void MainWindow::toggleFullscreen() {
    m_isFullscreen = !m_isFullscreen;
    
    if (m_isFullscreen) {
        // Enter borderless fullscreen
        SDL_SetWindowFullscreen(m_window, SDL_WINDOW_FULLSCREEN_DESKTOP);
        spdlog::info("Entered fullscreen mode");
    } else {
        // Exit fullscreen
        SDL_SetWindowFullscreen(m_window, 0);
        spdlog::info("Exited fullscreen mode");
    }
}

void MainWindow::shutdown() {
    if (!m_isInitialized) return;

    spdlog::info("Shutting down MainWindow");
    stopTelemetry();
    
    // Stop all camera streams first
    if (m_cameraManager) {
        m_cameraManager->stopAll();
    }
    
    // Save settings before shutdown
    auto& settings = SettingsManager::instance();
    
    if (!m_isFullscreen) {
        // Save window position and size (only when not in fullscreen)
        int x, y;
        SDL_GetWindowPosition(m_window, &x, &y);
        settings.setLastWindowPosition(x, y);
        settings.setLastWindowSize(m_windowWidth, m_windowHeight);
    }
    
    settings.save();

    m_cameraManager.reset();
    m_consoleWindow.reset();
    m_statsPanel.reset();
    m_cameraGrid.reset();

    if (m_renderer) {
        SDL_DestroyRenderer(m_renderer);
        m_renderer = nullptr;
    }

    if (m_window) {
        SDL_DestroyWindow(m_window);
        m_window = nullptr;
    }

    FontManager::instance().shutdown();
    SDL_Quit();
    m_isInitialized = false;
}

void MainWindow::startTelemetry() {
    if (m_telemetryRunning) return;
    m_telemetryRunning = true;
    m_telemetryThread = std::thread(&MainWindow::telemetryLoop, this);
}

void MainWindow::stopTelemetry() {
    m_telemetryRunning = false;
    if (m_telemetryThread.joinable()) {
        m_telemetryThread.join();
    }
}

float MainWindow::pingCameraMs() {
    if (m_pingCameraIp.empty()) {
        return 0.0f;
    }

    // Use system ping for simplicity; timeout 1s
    int timeoutSec = 1;
    std::string cmd = "ping -c 1 -W " + std::to_string(timeoutSec) + " " + m_pingCameraIp;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        return 0.0f;
    }

    char buffer[256];
    std::string output;
    while (fgets(buffer, sizeof(buffer), pipe)) {
        output += buffer;
    }
    pclose(pipe);

    std::regex timeRegex("time=([0-9.]+) ms");
    std::smatch match;
    if (std::regex_search(output, match, timeRegex) && match.size() > 1) {
        try {
            return std::stof(match[1]);
        } catch (...) {
            return 0.0f;
        }
    }

    return 0.0f; // fallback if ping failed
}

float MainWindow::sampleProcessCpuPercent() {
    std::ifstream statFile("/proc/self/stat");
    if (!statFile.is_open()) return 0.0f;

    std::string line;
    std::getline(statFile, line);
    std::istringstream iss(line);

    // Fields: pid (1), comm (2), state (3), ... utime (14), stime (15), cutime (16), cstime (17)
    std::string tmp;
    long utime = 0, stime = 0, cutime = 0, cstime = 0;
    for (int i = 1; i <= 17; ++i) {
        iss >> tmp;
        if (i == 14) utime = std::stol(tmp);
        if (i == 15) stime = std::stol(tmp);
        if (i == 16) cutime = std::stol(tmp);
        if (i == 17) cstime = std::stol(tmp);
    }

    uint64_t procJiffies = static_cast<uint64_t>(utime + stime + cutime + cstime);

    // Total jiffies from /proc/stat
    std::ifstream totalFile("/proc/stat");
    if (!totalFile.is_open()) return 0.0f;
    std::string cpuLine;
    std::getline(totalFile, cpuLine);
    std::istringstream cpuStream(cpuLine);
    cpuStream >> tmp; // skip "cpu"
    uint64_t user = 0, nice = 0, system = 0, idle = 0, iowait = 0, irq = 0, softirq = 0, steal = 0;
    cpuStream >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
    uint64_t totalJiffies = user + nice + system + idle + iowait + irq + softirq + steal;

    float percent = 0.0f;
    if (m_hasPrevCpuSample) {
        uint64_t deltaProc = procJiffies - m_prevProcJiffies;
        uint64_t deltaTotal = totalJiffies - m_prevTotalJiffies;
        if (deltaTotal > 0) {
            long cpus = sysconf(_SC_NPROCESSORS_ONLN);
            if (cpus < 1) cpus = 1;
            percent = (static_cast<float>(deltaProc) / static_cast<float>(deltaTotal)) * 100.0f / cpus;
        }
    }

    m_prevProcJiffies = procJiffies;
    m_prevTotalJiffies = totalJiffies;
    m_hasPrevCpuSample = true;
    return percent;
}

float MainWindow::sampleProcessRamPercent() {
    long pageSize = sysconf(_SC_PAGESIZE);
    std::ifstream statm("/proc/self/statm");
    if (!statm.is_open()) return 0.0f;
    long size = 0, resident = 0;
    statm >> size >> resident;
    uint64_t rssBytes = static_cast<uint64_t>(resident) * static_cast<uint64_t>(pageSize);

    std::ifstream meminfo("/proc/meminfo");
    if (!meminfo.is_open()) return 0.0f;
    std::string key;
    uint64_t memTotalKb = 0;
    while (meminfo >> key) {
        if (key == "MemTotal:") {
            meminfo >> memTotalKb;
            break;
        }
        // skip rest of line
        std::getline(meminfo, key);
    }
    if (memTotalKb == 0) return 0.0f;

    float memTotalBytes = static_cast<float>(memTotalKb) * 1024.0f;
    return (static_cast<float>(rssBytes) / memTotalBytes) * 100.0f;
}

float MainWindow::sampleGpuUsagePercent() {
    // Use nvidia-smi if available; query first GPU utilization
    const char* cmd = "nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits -i 0";
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return 0.0f;

    char buffer[128];
    std::string output;
    if (fgets(buffer, sizeof(buffer), pipe)) {
        output = buffer;
    }
    pclose(pipe);

    try {
        // Trim whitespace
        output.erase(0, output.find_first_not_of(" \t\n\r"));
        output.erase(output.find_last_not_of(" \t\n\r") + 1);
        if (!output.empty()) {
            return std::stof(output);
        }
    } catch (...) {
        return 0.0f;
    }
    return 0.0f;
}

void MainWindow::telemetryLoop() {
    using namespace std::chrono_literals;
    while (m_telemetryRunning) {
        float cpu = sampleProcessCpuPercent();
        float ram = sampleProcessRamPercent();
        float pingMs = pingCameraMs();
        float gpu = sampleGpuUsagePercent();

        m_cpuUsageAtomic.store(cpu);
        m_ramUsageAtomic.store(ram);
        m_latencyAtomic.store(pingMs);
        m_gpuUsageAtomic.store(gpu);

        std::this_thread::sleep_for(1s);
    }
}

} // namespace camera_viewer
