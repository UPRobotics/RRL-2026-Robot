#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <alsa/asoundlib.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>

namespace mic_pkg {

class MicTransmitter : public rclcpp::Node {
public:
    MicTransmitter();
    ~MicTransmitter() override;

private:
    // ALSA capture
    bool openDevice();
    void closeDevice();
    void captureLoop();

    // ROS2 publisher
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

    // ALSA handle
    snd_pcm_t* capture_handle_ = nullptr;

    // Parameters
    std::string device_;         // ALSA device name (e.g. "hw:2,0")
    unsigned int sample_rate_;   // Sample rate in Hz
    unsigned int channels_;      // Number of channels
    snd_pcm_uframes_t frames_;  // Frames per period (chunk size)

    // Capture thread
    std::thread capture_thread_;
    std::atomic<bool> running_{false};
};

}  // namespace mic_pkg
