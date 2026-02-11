#include "camera_pkg/console_sink.h"
#include "camera_pkg/console_window.h"
#include "camera_pkg/ui_helpers.h"
#include <spdlog/spdlog.h>

namespace camera_viewer {

ConsoleSink::ConsoleSink(ConsoleWindow* consoleWindow)
    : m_consoleWindow(consoleWindow)
{
}

void ConsoleSink::sink_it_(const spdlog::details::log_msg& msg) {
    if (!m_consoleWindow) return;

    spdlog::memory_buf_t formatted;
    formatter_->format(msg, formatted);
    std::string message(formatted.data(), formatted.size());
    
    if (!message.empty() && message.back() == '\n') {
        message.pop_back();
    }

    SDL_Color color;
    switch (msg.level) {
        case spdlog::level::trace:
        case spdlog::level::debug:
            color = Colors::CONSOLE_DEBUG;
            break;
        case spdlog::level::info:
            color = Colors::CONSOLE_INFO;
            break;
        case spdlog::level::warn:
            color = Colors::CONSOLE_WARNING;
            break;
        case spdlog::level::err:
        case spdlog::level::critical:
            color = Colors::CONSOLE_ERROR;
            break;
        default:
            color = Colors::CONSOLE_TEXT;
            break;
    }

    m_consoleWindow->addLog(message, color);
}

void ConsoleSink::flush_() {
}

} // namespace camera_viewer
