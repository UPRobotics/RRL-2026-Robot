#include "mic_pkg/transcript_window.h"
#include <algorithm>

namespace mic_pkg {

TranscriptWindow::TranscriptWindow(int width, int height, int font_size)
    : width_(width), height_(height), font_size_(font_size)
{
    running_ = true;
    render_thread_ = std::thread(&TranscriptWindow::renderLoop, this);
}

TranscriptWindow::~TranscriptWindow()
{
    running_ = false;
    if (render_thread_.joinable()) {
        render_thread_.join();
    }
}

void TranscriptWindow::setPartial(const std::string& text)
{
    std::lock_guard<std::mutex> lock(text_mutex_);
    partial_text_ = text;
}

void TranscriptWindow::addFinal(const std::string& text)
{
    std::lock_guard<std::mutex> lock(text_mutex_);
    final_lines_.push_back(text);
    if (final_lines_.size() > MAX_LINES) {
        final_lines_.pop_front();
    }
    partial_text_.clear();
}

void TranscriptWindow::setLanguage(const std::string& lang)
{
    std::lock_guard<std::mutex> lock(text_mutex_);
    current_lang_ = lang;
}

SDL_Rect TranscriptWindow::getButtonRect() const
{
    const int bw = 60, bh = 30, margin = 10;
    return {width_ - bw - margin, margin, bw, bh};
}

void TranscriptWindow::renderLoop()
{
    SDL_Init(SDL_INIT_VIDEO);
    TTF_Init();

    window_ = SDL_CreateWindow("Speech Recognition",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        width_, height_, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    font_ = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", font_size_);
    button_font_ = TTF_OpenFont("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 16);

    if (!font_) {
        font_ = TTF_OpenFont("/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf", font_size_);
    }
    if (!button_font_) {
        button_font_ = TTF_OpenFont("/usr/share/fonts/truetype/noto/NotoSansMono-Regular.ttf", 16);
    }

    while (running_) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running_ = false;
            }
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_RESIZED) {
                width_ = event.window.data1;
                height_ = event.window.data2;
            }
            if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
                SDL_Rect btn = getButtonRect();
                int mx = event.button.x, my = event.button.y;
                if (mx >= btn.x && mx <= btn.x + btn.w &&
                    my >= btn.y && my <= btn.y + btn.h) {
                    // Toggle language
                    std::string new_lang;
                    {
                        std::lock_guard<std::mutex> lock(text_mutex_);
                        new_lang = (current_lang_ == "EN") ? "ES" : "EN";
                        current_lang_ = new_lang;
                    }
                    if (lang_callback_) {
                        lang_callback_(new_lang);
                    }
                }
            }
        }

        SDL_SetRenderDrawColor(renderer_, 30, 30, 30, 255);
        SDL_RenderClear(renderer_);

        renderButton();
        renderText();

        SDL_RenderPresent(renderer_);
        SDL_Delay(33);  // ~30 fps
    }

    if (button_font_) TTF_CloseFont(button_font_);
    if (font_) TTF_CloseFont(font_);
    if (renderer_) SDL_DestroyRenderer(renderer_);
    if (window_) SDL_DestroyWindow(window_);
    TTF_Quit();
    SDL_Quit();
}

void TranscriptWindow::renderButton()
{
    if (!button_font_) return;

    std::lock_guard<std::mutex> lock(text_mutex_);

    SDL_Rect btn = getButtonRect();

    // Button background
    SDL_SetRenderDrawColor(renderer_, 60, 120, 200, 255);
    SDL_RenderFillRect(renderer_, &btn);

    // Button border
    SDL_SetRenderDrawColor(renderer_, 100, 160, 240, 255);
    SDL_RenderDrawRect(renderer_, &btn);

    // Button text
    SDL_Color white = {255, 255, 255, 255};
    SDL_Surface* surface = TTF_RenderUTF8_Blended(button_font_, current_lang_.c_str(), white);
    if (surface) {
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
        SDL_Rect dst = {
            btn.x + (btn.w - surface->w) / 2,
            btn.y + (btn.h - surface->h) / 2,
            surface->w, surface->h
        };
        SDL_RenderCopy(renderer_, texture, nullptr, &dst);
        SDL_DestroyTexture(texture);
        SDL_FreeSurface(surface);
    }
}

void TranscriptWindow::renderText()
{
    if (!font_) return;

    std::lock_guard<std::mutex> lock(text_mutex_);

    const int line_height = font_size_ + 4;
    const int padding = 10;
    const int max_visible_lines = (height_ - padding * 2) / line_height;

    SDL_Color white = {255, 255, 255, 255};
    SDL_Color green = {150, 200, 150, 255};

    // Reserve 1 line for partial text if present
    int lines_for_final = max_visible_lines;
    if (!partial_text_.empty()) {
        lines_for_final = std::max(1, max_visible_lines - 1);
    }

    int start_idx = 0;
    int total_final = static_cast<int>(final_lines_.size());
    if (total_final > lines_for_final) {
        start_idx = total_final - lines_for_final;
    }

    int y = height_ - padding - line_height;

    // Draw partial text at the very bottom
    if (!partial_text_.empty()) {
        SDL_Surface* surface = TTF_RenderUTF8_Blended(font_, partial_text_.c_str(), green);
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
            SDL_Rect dst = {padding, y, surface->w, surface->h};
            if (dst.w > width_ - padding * 2) dst.w = width_ - padding * 2;
            SDL_RenderCopy(renderer_, texture, nullptr, &dst);
            SDL_DestroyTexture(texture);
            SDL_FreeSurface(surface);
        }
        y -= line_height;
    }

    // Draw final lines from bottom to top
    for (int i = total_final - 1; i >= start_idx && y >= padding; --i) {
        SDL_Surface* surface = TTF_RenderUTF8_Blended(font_, final_lines_[i].c_str(), white);
        if (surface) {
            SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer_, surface);
            SDL_Rect dst = {padding, y, surface->w, surface->h};
            if (dst.w > width_ - padding * 2) dst.w = width_ - padding * 2;
            SDL_RenderCopy(renderer_, texture, nullptr, &dst);
            SDL_DestroyTexture(texture);
            SDL_FreeSurface(surface);
        }
        y -= line_height;
    }
}

}  // namespace mic_pkg
