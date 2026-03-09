#pragma once

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <string>
#include <deque>
#include <mutex>
#include <thread>
#include <atomic>
#include <functional>

namespace mic_pkg {

class TranscriptWindow {
public:
    using LanguageCallback = std::function<void(const std::string& lang)>;

    TranscriptWindow(int width = 600, int height = 300, int font_size = 18);
    ~TranscriptWindow();

    // Set the current partial (in-progress) text
    void setPartial(const std::string& text);

    // Append a finalized line of text
    void addFinal(const std::string& text);

    // Set the current language label ("EN" or "ES")
    void setLanguage(const std::string& lang);

    // Register callback when language button is clicked
    void setLanguageCallback(LanguageCallback cb) { lang_callback_ = std::move(cb); }

    bool isRunning() const { return running_; }

private:
    void renderLoop();
    void renderText();
    void renderButton();
    SDL_Rect getButtonRect() const;

    SDL_Window* window_ = nullptr;
    SDL_Renderer* renderer_ = nullptr;
    TTF_Font* font_ = nullptr;
    TTF_Font* button_font_ = nullptr;

    int width_;
    int height_;
    int font_size_;

    std::string partial_text_;
    std::deque<std::string> final_lines_;
    static constexpr size_t MAX_LINES = 50;

    std::string current_lang_ = "EN";
    LanguageCallback lang_callback_;

    std::mutex text_mutex_;
    std::thread render_thread_;
    std::atomic<bool> running_{false};
};

}  // namespace mic_pkg
