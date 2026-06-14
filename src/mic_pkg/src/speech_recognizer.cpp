#include "mic_pkg/speech_recognizer.h"
#include <cstring>

namespace mic_pkg {

SpeechRecognizer::SpeechRecognizer(const std::string& model_path, float sample_rate)
{
    vosk_set_log_level(-1);  // Suppress Vosk/Kaldi logs

    model_ = vosk_model_new(model_path.c_str());
    if (!model_) {
        return;
    }

    recognizer_ = vosk_recognizer_new(model_, sample_rate);
    if (!recognizer_) {
        vosk_model_free(model_);
        model_ = nullptr;
    }
}

SpeechRecognizer::~SpeechRecognizer()
{
    if (recognizer_) vosk_recognizer_free(recognizer_);
    if (model_) vosk_model_free(model_);
}

void SpeechRecognizer::feedAudio(const char* data, int length)
{
    if (!recognizer_ || !callback_) return;

    int finished = vosk_recognizer_accept_waveform(recognizer_, data, length);

    if (finished) {
        // Complete utterance
        const char* result = vosk_recognizer_result(recognizer_);
        std::string text = extractText(result, "text");
        if (!text.empty()) {
            callback_(text, true);
        }
    } else {
        // Partial result (live preview)
        const char* partial = vosk_recognizer_partial_result(recognizer_);
        std::string text = extractText(partial, "partial");
        if (!text.empty()) {
            callback_(text, false);
        }
    }
}

std::string SpeechRecognizer::extractText(const char* json, const char* key) const
{
    // Minimal JSON parsing — find "key" : "value"
    if (!json) return {};

    std::string search = std::string("\"") + key + "\" : \"";
    const char* pos = strstr(json, search.c_str());
    if (!pos) return {};

    pos += search.length();
    const char* end = strchr(pos, '"');
    if (!end) return {};

    return std::string(pos, end);
}

}  // namespace mic_pkg
