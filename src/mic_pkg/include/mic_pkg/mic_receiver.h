#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <alsa/asoundlib.h>
#include <string>
#include <memory>
#include <mutex>
#include "mic_pkg/speech_recognizer.h"
#include "mic_pkg/transcript_window.h"

namespace mic_pkg {

class MicReceiver : public rclcpp::Node {
public:
    MicReceiver();
    ~MicReceiver() override;

private:
    // ALSA playback
    bool openDevice();
    void closeDevice();
    void prefillSilence();
    void audioCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

    // ROS2 subscriber & publisher
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_publisher_;

    // ALSA handle
    snd_pcm_t* playback_handle_ = nullptr;

    // Model switching
    void switchLanguage(const std::string& lang);
    void setupRecognizerCallback();

    // Parameters
    std::string device_;
    unsigned int sample_rate_;
    unsigned int channels_;
    snd_pcm_uframes_t frames_;
    std::string model_path_en_;
    std::string model_path_es_;
    std::string current_lang_;

    // Deferred ALSA start
    bool needs_prepare_{true};

    // Speech recognition + display
    std::mutex recognizer_mutex_;
    std::unique_ptr<SpeechRecognizer> recognizer_;
    std::unique_ptr<TranscriptWindow> window_;
};

}  // namespace mic_pkg
