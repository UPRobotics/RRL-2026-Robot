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
    // USB device auto-detection
    std::string findDeviceByUsbId(const std::string& vendor_id, const std::string& product_id);
    std::string findPulseSource();

    // ALSA capture
    bool openDevice();
    void closeDevice();
    void captureLoop();

    // ROS2 publisher
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;

    // ALSA handle
    snd_pcm_t* capture_handle_ = nullptr;

    // Parameters
    std::string device_;         // ALSA device name (e.g. "hw:2,0") or "auto"
    std::string usb_vendor_id_;  // USB vendor ID for auto-detection
    std::string usb_product_id_; // USB product ID for auto-detection
    unsigned int sample_rate_;
    unsigned int channels_;
    snd_pcm_uframes_t frames_;

    // Capture thread
    std::thread capture_thread_;
    std::atomic<bool> running_{false};
};

}  // namespace mic_pkg
