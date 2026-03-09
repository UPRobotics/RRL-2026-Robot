#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <alsa/asoundlib.h>
#include <string>

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

    // ROS2 subscriber
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscription_;

    // ALSA handle
    snd_pcm_t* playback_handle_ = nullptr;

    // Parameters
    std::string device_;         // ALSA playback device (e.g. "default")
    unsigned int sample_rate_;
    unsigned int channels_;
    snd_pcm_uframes_t frames_;
};

}  // namespace mic_pkg
