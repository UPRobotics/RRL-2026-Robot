#pragma once

#include <string>
#include <functional>

// Vosk C API
extern "C" {
#include <vosk_api.h>
}

namespace mic_pkg {

class SpeechRecognizer {
public:
    // Callback types: partial results (live preview) and final results (complete utterance)
    using TextCallback = std::function<void(const std::string& text, bool is_final)>;

    SpeechRecognizer(const std::string& model_path, float sample_rate);
    ~SpeechRecognizer();

    // Feed raw PCM audio (S16_LE mono) and trigger callbacks
    void feedAudio(const char* data, int length);

    // Set callback for recognized text
    void setCallback(TextCallback cb) { callback_ = std::move(cb); }

    bool isValid() const { return recognizer_ != nullptr; }

private:
    std::string extractText(const char* json, const char* key) const;

    VoskModel* model_ = nullptr;
    VoskRecognizer* recognizer_ = nullptr;
    TextCallback callback_;
};

}  // namespace mic_pkg
