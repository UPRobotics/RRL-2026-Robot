#include "robot_pkg/VESC.hpp"
#include <algorithm>
#include <cstring>
#include <filesystem>

using namespace LibSerial;

VESC::VESC(uint8_t id, int baud, int to) : motor_id(id), baudrate(baud), timeout(to),logger(rclcpp::get_logger("VESC")) {
        serial_port_ = std::make_unique<SerialPort>();
      }

VESC::~VESC() {
    disconnect();
    if (serial_port_) {
        try { serial_port_->Close(); } catch(...) {}
        serial_port_.release();
    }
}

void VESC::setupPort() {
    if (baudrate == 115200) serial_port_->SetBaudRate(BaudRate::BAUD_115200);
    else serial_port_->SetBaudRate(BaudRate::BAUD_9600);

    serial_port_->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
    serial_port_->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
    serial_port_->SetParity(Parity::PARITY_NONE);
    serial_port_->SetStopBits(StopBits::STOP_BITS_1);
}

bool VESC::connect() {
    try {
        if (serial_port_->IsOpen()) serial_port_->Close();
        serial_port_->Open(port_name);
        setupPort();
        serial_port_->FlushIOBuffers();
        running = true;
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Connection failed: %s", e.what());
        return false;
    }
}

void VESC::disconnect() {
    running = false;
    if (!serial_port_) return;
    
    try {
        serial_port_->Close();
        RCLCPP_INFO(logger, "Disconnected.");
    } catch(...) {
        RCLCPP_WARN(logger, "Close failed during disconnect (USB gone?)");
    }
}

bool VESC::isConnected(){
    if (!running || !serial_port_)
        return false;
    try{
        return serial_port_->IsOpen();
    }catch(const std::exception& e){
        RCLCPP_DEBUG(logger, "IsOpen() threw exception: %s", e.what());
        return false;
    }catch(...){
        return false;
    }
}

std::vector<std::string> scanPorts(){
    std::vector<std::string> ports;
    for(auto const& entry : std::filesystem::directory_iterator("/dev")){
        std::string name = entry.path().string();

        if(name.find("ttyACM") != std::string::npos){
            ports.push_back(name);
        }
    }
    return ports;
}

bool VESC::autoConnect(){
    auto ports = scanPorts();

    for(const auto& port: ports){
        RCLCPP_INFO(logger,"detected the port %s", port.c_str());
        try{
            if (serial_port_) {
                try { serial_port_->Close(); } catch(...) {}
                serial_port_.release();
            }
            serial_port_ = std::make_unique<SerialPort>();
            serial_port_->Open(port);
            setupPort();

            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            try {
                serial_port_->FlushInputBuffer();
                serial_port_->FlushIOBuffers();
                RCLCPP_INFO(logger, "Buffers flushed.");
            } catch (const std::exception &e) {
                RCLCPP_WARN(logger, "Flush failed for %s: %s - skipping port", port.c_str(), e.what());
                try { serial_port_->Close(); } catch(...) {}
                serial_port_.release();
                continue;
            }

            VESCData data;
            running = true;

                if (get_telemetry(data)) {
                    if (data.motor_controller_id == motor_id) {
                        port_name = port;
                        return true;
                    }
                }

            running = false;
            try { serial_port_->Close(); } catch(...) {}
            serial_port_.release();

        }
        catch(const std::exception& e){
            RCLCPP_INFO(logger, "Port %s failed: %s", port.c_str(), e.what());
            running = false;
            try { if(serial_port_) serial_port_->Close(); } catch(...) {}
            serial_port_.release();
            continue;
        }
    }

    return false;
}

void VESC::set_rpm(int32_t rpm) {
    if (!isConnected()) return;

    try{
        std::vector<uint8_t> payload;
        payload.push_back(8); // COMM_SET_RPM
        payload.push_back((rpm >> 24) & 0xFF);
        payload.push_back((rpm >> 16) & 0xFF);
        payload.push_back((rpm >> 8) & 0xFF);
        payload.push_back(rpm & 0xFF);

        send_vesc_packet(payload);
    }catch(...){
        RCLCPP_ERROR(logger, "set_rpm unknown error");
        running = false;
        try { if(serial_port_) serial_port_->Close(); } catch(...) {}
    }
}

void VESC::send_vesc_packet(const std::vector<uint8_t> &payload) {
    if (!isConnected()) return;

    try{
        std::vector<uint8_t> packet;
        packet.push_back(2);
        packet.push_back(static_cast<uint8_t>(payload.size()));
        packet.insert(packet.end(), payload.begin(), payload.end());

        uint16_t crc = crc16(payload);
        packet.push_back((crc >> 8) & 0xFF);
        packet.push_back(crc & 0xFF);
        packet.push_back(3);

        serial_port_->Write(packet);
    }catch(...){
        RCLCPP_WARN(logger, "Write failed (unknown error)");
        running = false;
        try { if(serial_port_) serial_port_->Close(); } catch(...) {}
    }
}

uint16_t VESC::crc16(const std::vector<uint8_t>& data, uint16_t poly, uint16_t init_val){
    uint16_t crc = init_val;
    for (uint8_t byte : data) {
        crc ^= byte << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

std::vector<uint8_t> VESC::find_packet(const std::vector<uint8_t>& response){
    if (response.empty() || response.size() < 5) {
        return {};
    }
    
    for (size_t i = 0; i < response.size() - 2; i++) { 
        if (response[i] == 2) {
            uint8_t length = response[i + 1];
            size_t packet_end = i + 2 + length + 3;
            
            if (packet_end <= response.size() && response[packet_end - 1] == 3) {
                std::vector<uint8_t> data(response.begin() + i + 2, 
                                         response.begin() + i + 2 + length);
                
                uint16_t crc_received = (response[i + 2 + length] << 8) | 
                                       response[i + 2 + length + 1];
                uint16_t crc_calculated = crc16(data);
                
                if (crc_received == crc_calculated) {
                    return data;
                }
            }
        }
    }
        return {};
}

void VESC::request_values() {
    if (!isConnected()) return;

    std::vector<uint8_t> payload;
    payload.push_back(4); 
    send_vesc_packet(payload);
}

std::vector<uint8_t> VESC::read_bytes() { // TODO: Main change, check if it works with this one
    std::vector<uint8_t> buffer;
    if (!isConnected()) return buffer;

    try {
        // Read until we hit the end byte '3' or timeout
        char byte;
        while (true) {
            serial_port_->ReadByte(byte, timeout);
            buffer.push_back(static_cast<uint8_t>(byte));
            if (byte == 3 && buffer.size() > 5) break; 
        }
    } catch (const ReadTimeout&) {
        // Normal behavior if VESC is slow or packet ends
    } catch (...) {
        running = false;
    }
    return buffer;
}

bool VESC::get_telemetry(VESCData& out) {
    request_values();
    if (!running) return false;

    auto raw = read_bytes();
    auto payload = find_packet(raw);

    if (payload.empty() || payload[0] != 4) return false;

    // Offsets from official VESC firmware
    auto get_i16 = [&](int i) {
        return (payload[i] << 8) | payload[i+1];
    };

    auto get_i32 = [&](int i) {
        return (payload[i] << 24) |
               (payload[i+1] << 16) |
               (payload[i+2] << 8) |
                payload[i+3];
    };
    out.temp_fet      = get_i16(1) / 10.0f;
    out.current_motor = get_i32(5) / 100.0f;
    out.rpm           = get_i32(23);
    out.input_voltage = get_i16(29) / 10.0f;

    if(payload.size() > 58){
        out.motor_controller_id = payload[58];
    }
    return true;
}
