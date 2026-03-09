#include "mic_pkg/mic_receiver.h"

namespace mic_pkg {

MicReceiver::MicReceiver()
    : Node("mic_receiver")
{
    this->declare_parameter<std::string>("device", "default");
    this->declare_parameter<int>("sample_rate", 48000);
    this->declare_parameter<int>("channels", 1);
    this->declare_parameter<int>("frames_per_period", 512);
    this->declare_parameter<std::string>("model_path_en",
        "/home/chumbi/roboticaWS/models/vosk-model-small-en-us-0.15");
    this->declare_parameter<std::string>("model_path_es",
        "/home/chumbi/roboticaWS/models/vosk-model-small-es-0.42");
    this->declare_parameter<std::string>("lang", "en");

    device_ = this->get_parameter("device").as_string();
    sample_rate_ = static_cast<unsigned int>(this->get_parameter("sample_rate").as_int());
    channels_ = static_cast<unsigned int>(this->get_parameter("channels").as_int());
    frames_ = static_cast<snd_pcm_uframes_t>(this->get_parameter("frames_per_period").as_int());
    model_path_en_ = this->get_parameter("model_path_en").as_string();
    model_path_es_ = this->get_parameter("model_path_es").as_string();
    current_lang_ = this->get_parameter("lang").as_string();

    RCLCPP_INFO(this->get_logger(), "Mic receiver starting — device=%s rate=%u ch=%u frames=%lu",
                device_.c_str(), sample_rate_, channels_, frames_);

    if (!openDevice()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ALSA playback device '%s'", device_.c_str());
        return;
    }

    subscription_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "audio_raw", 10,
        std::bind(&MicReceiver::audioCallback, this, std::placeholders::_1));

    text_publisher_ = this->create_publisher<std_msgs::msg::String>("speech_text", 10);

    // Create transcript window first
    window_ = std::make_unique<TranscriptWindow>(600, 300, 18);
    window_->setLanguage(current_lang_ == "es" ? "ES" : "EN");
    window_->setLanguageCallback([this](const std::string& lang) {
        switchLanguage(lang == "ES" ? "es" : "en");
    });

    // Load initial model
    switchLanguage(current_lang_);

    RCLCPP_INFO(this->get_logger(), "Subscribed to audio_raw topic");
}

MicReceiver::~MicReceiver()
{
    window_.reset();
    std::lock_guard<std::mutex> lock(recognizer_mutex_);
    recognizer_.reset();
    closeDevice();
}

void MicReceiver::switchLanguage(const std::string& lang)
{
    const std::string& model_path = (lang == "es") ? model_path_es_ : model_path_en_;

    RCLCPP_INFO(this->get_logger(), "Switching to %s model: %s",
                lang.c_str(), model_path.c_str());

    auto new_recognizer = std::make_unique<SpeechRecognizer>(
        model_path, static_cast<float>(sample_rate_));

    if (!new_recognizer->isValid()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load Vosk model from '%s'", model_path.c_str());
        return;
    }

    current_lang_ = lang;

    // Set up callback on new recognizer
    new_recognizer->setCallback([this](const std::string& text, bool is_final) {
        if (is_final) {
            if (window_) window_->addFinal(text);
            auto msg = std_msgs::msg::String();
            msg.data = text;
            text_publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "[FINAL] %s", text.c_str());
        } else {
            if (window_) window_->setPartial(text);
        }
    });

    {
        std::lock_guard<std::mutex> lock(recognizer_mutex_);
        recognizer_ = std::move(new_recognizer);
    }

    if (window_) {
        window_->setLanguage(lang == "es" ? "ES" : "EN");
    }

    RCLCPP_INFO(this->get_logger(), "Now using %s model", lang.c_str());
}

bool MicReceiver::openDevice()
{
    int err;

    err = snd_pcm_open(&playback_handle_, device_.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "snd_pcm_open: %s", snd_strerror(err));
        return false;
    }

    snd_pcm_hw_params_t* hw_params;
    snd_pcm_hw_params_alloca(&hw_params);

    err = snd_pcm_hw_params_any(playback_handle_, hw_params);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params_any: %s", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params_set_access(playback_handle_, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_access: %s", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params_set_format(playback_handle_, hw_params, SND_PCM_FORMAT_S16_LE);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_format: %s", snd_strerror(err));
        return false;
    }

    unsigned int requested_rate = sample_rate_;
    err = snd_pcm_hw_params_set_rate_near(playback_handle_, hw_params, &sample_rate_, nullptr);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_rate: %s", snd_strerror(err));
        return false;
    }
    if (sample_rate_ != requested_rate) {
        RCLCPP_WARN(this->get_logger(), "Requested rate %u, device negotiated %u",
                    requested_rate, sample_rate_);
    }

    err = snd_pcm_hw_params_set_channels(playback_handle_, hw_params, channels_);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_channels: %s", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params_set_period_size_near(playback_handle_, hw_params, &frames_, nullptr);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_period_size: %s", snd_strerror(err));
        return false;
    }

    // Use 4 periods to absorb ROS message jitter
    unsigned int periods = 4;
    err = snd_pcm_hw_params_set_periods_near(playback_handle_, hw_params, &periods, nullptr);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "set_periods: %s", snd_strerror(err));
        return false;
    }

    err = snd_pcm_hw_params(playback_handle_, hw_params);
    if (err < 0) {
        RCLCPP_ERROR(this->get_logger(), "snd_pcm_hw_params: %s", snd_strerror(err));
        return false;
    }

    // Query actual buffer size
    snd_pcm_uframes_t buffer_size;
    snd_pcm_hw_params_get_buffer_size(hw_params, &buffer_size);

    RCLCPP_INFO(this->get_logger(), "ALSA playback device opened: rate=%u ch=%u period=%lu periods=%u buffer=%lu",
                sample_rate_, channels_, frames_, periods, buffer_size);

    return true;
}

void MicReceiver::closeDevice()
{
    if (playback_handle_) {
        snd_pcm_drop(playback_handle_);
        snd_pcm_close(playback_handle_);
        playback_handle_ = nullptr;
    }
}

void MicReceiver::prefillSilence()
{
    if (!playback_handle_) return;

    // Write 2 periods of silence to pre-fill the buffer
    const size_t bytes_per_frame = channels_ * 2;
    std::vector<uint8_t> silence(frames_ * bytes_per_frame * 2, 0);
    snd_pcm_writei(playback_handle_, silence.data(), frames_ * 2);
}

void MicReceiver::audioCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    if (!playback_handle_ || msg->data.empty()) {
        return;
    }

    // On first audio or after recovery, prepare and prefill
    if (needs_prepare_) {
        snd_pcm_prepare(playback_handle_);
        prefillSilence();
        needs_prepare_ = false;
    }

    const size_t bytes_per_frame = channels_ * 2;  // S16_LE = 2 bytes per sample
    const snd_pcm_uframes_t frame_count = msg->data.size() / bytes_per_frame;

    snd_pcm_sframes_t written = snd_pcm_writei(
        playback_handle_, msg->data.data(), frame_count);

    if (written < 0) {
        RCLCPP_WARN(this->get_logger(), "ALSA write error: %s — recovering",
                    snd_strerror(static_cast<int>(written)));
        needs_prepare_ = true;
    }

    // Feed audio to speech recognizer
    {
        std::lock_guard<std::mutex> lock(recognizer_mutex_);
        if (recognizer_) {
            recognizer_->feedAudio(
                reinterpret_cast<const char*>(msg->data.data()),
                static_cast<int>(msg->data.size()));
        }
    }
}

}  // namespace mic_pkg
