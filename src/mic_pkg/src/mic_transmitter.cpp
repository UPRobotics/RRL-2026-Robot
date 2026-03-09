#include "mic_pkg/mic_transmitter.h"

namespace mic_pkg {

MicTransmitter::MicTransmitter()
    : Node("mic_transmitter")
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("device", "hw:2,0");
    this->declare_parameter<int>("sample_rate", 16000);
    this->declare_parameter<int>("channels", 1);
    this->declare_parameter<int>("frames_per_period", 1024);

    device_ = this->get_parameter("device").as_string();
    sample_rate_ = static_cast<unsigned int>(this->get_parameter("sample_rate").as_int());
    channels_ = static_cast<unsigned int>(this->get_parameter("channels").as_int());
    frames_ = static_cast<snd_pcm_uframes_t>(this->get_parameter("frames_per_period").as_int());

    RCLCPP_INFO(this->get_logger(), "Mic transmitter starting — device=%s rate=%u ch=%u frames=%lu",
                device_.c_str(), sample_rate_, channels_, frames_);

    // Create publisher for raw audio chunks
    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("audio_raw", 10);

    if (!openDevice()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ALSA capture device '%s'", device_.c_str());
        return;
    }

    // Start capture thread
    running_ = true;
    capture_thread_ = std::thread(&MicTransmitter::captureLoop, this);
}

MicTransmitter::~MicTransmitter()
{
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    closeDevice();
}

bool MicTransmitter::openDevice()
{
    int err;

    // Open PCM device for capture
    err = snd_pcm_open(&capture_handle_, device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "snd_pcm_open: %s", snd_strerror(err));
        return false;
    }

    // Configure hardware parameters
    snd_pcm_hw_params_t* hw_params;
    snd_pcm_hw_params_alloca(&hw_params);

    err = snd_pcm_hw_params_any(capture_handle_, hw_params);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params_any: %s", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params_set_access(capture_handle_, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_access: %s", snd_strerror(err));
        return false;
    }

    // 16-bit signed little-endian
    err = snd_pcm_hw_params_set_format(capture_handle_, hw_params, SND_PCM_FORMAT_S16_LE);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_format: %s", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params_set_rate_near(capture_handle_, hw_params, &sample_rate_, nullptr);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_rate: %s", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params_set_channels(capture_handle_, hw_params, channels_);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_channels: %s", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params_set_period_size_near(capture_handle_, hw_params, &frames_, nullptr);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_period_size: %s", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params(capture_handle_, hw_params);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params: %s", snd_strerror(err));
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "ALSA device opened: rate=%u ch=%u period=%lu",
                sample_rate_, channels_, frames_);
    return true;
}

void MicTransmitter::closeDevice()
{
    if (capture_handle_) {
        snd_pcm_close(capture_handle_);
        capture_handle_ = nullptr;
    }
}

void MicTransmitter::captureLoop()
{
    // Buffer size: frames * channels * 2 bytes (S16_LE)
    const size_t bytes_per_frame = channels_ * 2;
    const size_t buffer_size = frames_ * bytes_per_frame;
    std::vector<uint8_t> buffer(buffer_size);

    RCLCPP_INFO(this->get_logger(), "Capture loop started (buffer=%zu bytes)", buffer_size);

    while (running_ && rclcpp::ok()) {
        snd_pcm_sframes_t frames_read = snd_pcm_readi(
            capture_handle_, buffer.data(), frames_);

        if (frames_read < 0) {
            RCLCPP_WARN(this->get_logger(), "ALSA read error: %s — recovering",
                        snd_strerror(static_cast<int>(frames_read)));
            snd_pcm_prepare(capture_handle_);
            continue;
        }

        // Publish the captured audio chunk
        auto msg = std_msgs::msg::UInt8MultiArray();
        const size_t actual_bytes = static_cast<size_t>(frames_read) * bytes_per_frame;
        msg.data.assign(buffer.begin(), buffer.begin() + actual_bytes);
        publisher_->publish(msg);
    }

    RCLCPP_INFO(this->get_logger(), "Capture loop stopped");
}

}  // namespace mic_pkg
