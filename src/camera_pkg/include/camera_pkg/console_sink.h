#pragma once

#include <spdlog/sinks/base_sink.h>
#include <mutex>

namespace camera_viewer {

class ConsoleWindow;

/**
 * @brief Custom spdlog sink that forwards all log messages to the console window
 */
class ConsoleSink : public spdlog::sinks::base_sink<std::mutex> {
public:
    explicit ConsoleSink(ConsoleWindow* consoleWindow);
    
protected:
    void sink_it_(const spdlog::details::log_msg& msg) override;
    void flush_() override;

private:
    ConsoleWindow* m_consoleWindow;
};

} // namespace camera_viewer
