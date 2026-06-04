#include "mic_pkg/mic_transmitter.h"
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace mic_pkg {

std::string MicTransmitter::findDeviceByUsbId(const std::string& vendor_id,
                                               const std::string& product_id)
{
    // Scan /sys/class/sound/cardN, resolve the device symlink,
    // walk parent directories looking for idVendor/idProduct match.
    const fs::path snd_class("/sys/class/sound");

    for (auto& entry : fs::directory_iterator(snd_class)) {
        std::string name = entry.path().filename().string();
        if (name.substr(0, 4) != "card") continue;
        if (name.find('D') != std::string::npos) continue; // skip cardXDY (pcm devices)

        int card_num = -1;
        try { card_num = std::stoi(name.substr(4)); }
        catch (...) { continue; }

        // Resolve the device symlink to get the real sysfs path
        fs::path dev_link = entry.path() / "device";
        if (!fs::exists(dev_link)) continue;

        fs::path real_path;
        try { real_path = fs::canonical(dev_link); }
        catch (...) { continue; }

        // Walk up parent directories looking for idVendor/idProduct
        fs::path search = real_path;
        while (search != search.root_path()) {
            fs::path vf = search / "idVendor";
            fs::path pf = search / "idProduct";
            if (fs::exists(vf) && fs::exists(pf)) {
                std::ifstream vfs(vf), pfs(pf);
                std::string vid, pid;
                std::getline(vfs, vid);
                std::getline(pfs, pid);
                // Trim whitespace
                while (!vid.empty() && std::isspace(vid.back())) vid.pop_back();
                while (!pid.empty() && std::isspace(pid.back())) pid.pop_back();

                if (vid == vendor_id && pid == product_id) {
                    std::string device = "hw:" + std::to_string(card_num) + ",0";
                    RCLCPP_INFO(this->get_logger(),
                        "Auto-detected USB device %s:%s as ALSA %s (card %d)",
                        vendor_id.c_str(), product_id.c_str(),
                        device.c_str(), card_num);
                    return device;
                }
                break;  // Found USB IDs but didn't match — stop walking up
            }
            search = search.parent_path();
        }
    }
    return {};  // Not found
}

std::string MicTransmitter::findPulseSource()
{
    FILE* pipe = popen("pactl list sources short 2>/dev/null", "r");
    if (!pipe) return {};

    char line[512];
    std::string result;
    while (fgets(line, sizeof(line), pipe)) {
        std::string s(line);
        if (s.find("alsa_input.usb-") != std::string::npos) {
            std::istringstream iss(s);
            std::string index, name;
            iss >> index >> name;
            if (!name.empty()) {
                result = "pulse:" + name;
                break;
            }
        }
    }
    pclose(pipe);
    return result;
}

MicTransmitter::MicTransmitter()
    : Node("mic_transmitter")
{
    // Declare parameters with defaults
    this->declare_parameter<std::string>("device", "auto");
    this->declare_parameter<std::string>("usb_vendor_id", "4c4a");
    this->declare_parameter<std::string>("usb_product_id", "4155");
    this->declare_parameter<int>("sample_rate", 48000);
    this->declare_parameter<int>("channels", 1);
    this->declare_parameter<int>("frames_per_period", 512);

    device_ = this->get_parameter("device").as_string();
    usb_vendor_id_ = this->get_parameter("usb_vendor_id").as_string();
    usb_product_id_ = this->get_parameter("usb_product_id").as_string();
    sample_rate_ = static_cast<unsigned int>(this->get_parameter("sample_rate").as_int());
    channels_ = static_cast<unsigned int>(this->get_parameter("channels").as_int());
    frames_ = static_cast<snd_pcm_uframes_t>(this->get_parameter("frames_per_period").as_int());

    // Auto-detect device by USB ID
    if (device_ == "auto") {
        device_ = findDeviceByUsbId(usb_vendor_id_, usb_product_id_);
        if (device_.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                "USB microphone %s:%s not found — is it connected?",
                usb_vendor_id_.c_str(), usb_product_id_.c_str());
            return;
        }
    }

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

    // Open PCM device for capture; if busy (PulseAudio/PipeWire holds hw device),
    // fall back to opening through PulseAudio ALSA plugin.
    err = snd_pcm_open(&capture_handle_, device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
    if (err == -EBUSY) {
        std::string pa_device = findPulseSource();
        if (!pa_device.empty()) {
            RCLCPP_INFO(this->get_logger(),
                "hw device busy — falling back to PulseAudio source: %s", pa_device.c_str());
            device_ = pa_device;
            err = snd_pcm_open(&capture_handle_, device_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
        }
    }
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

    unsigned int requested_rate = sample_rate_;
    err = snd_pcm_hw_params_set_rate_near(capture_handle_, hw_params, &sample_rate_, nullptr);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_rate: %s", snd_strerror(err));
        return false;
    }
    if (sample_rate_ != requested_rate) {
        RCLCPP_WARN(this->get_logger(), "Requested rate %u, device negotiated %u",
                    requested_rate, sample_rate_);
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

    // Use 2 periods for low latency
    unsigned int periods = 2;
    err = snd_pcm_hw_params_set_periods_near(capture_handle_, hw_params, &periods, nullptr);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_periods: %s", snd_strerror(err));
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
