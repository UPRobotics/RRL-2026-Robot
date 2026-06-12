#include "robot_pkg/VESC.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <cerrno>
#include <mutex>

using namespace LibSerial;

std::mutex VESC::scan_mutex_;
std::atomic<bool> VESC::s_suppress_logs_{false};



VESC::VESC(uint8_t id, int baud, int to) : motor_id(id), baudrate(baud), timeout(to),logger(rclcpp::get_logger("VESC")) {
        serial_port_ = std::make_unique<SerialPort>();
      }

VESC::~VESC() {
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    disconnect();
    if (serial_port_) {
        try { serial_port_->Close(); } catch(...) {}
        serial_port_.reset();
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
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    try {
        if (serial_port_->IsOpen()) serial_port_->Close();
        serial_port_->Open(port_name);
        setupPort();
        serial_port_->FlushIOBuffers();
        running = true;
        return true;
    } catch (const std::exception& e) {
        if (!VESC::getSuppressLogs()) RCLCPP_ERROR(logger, "Connection failed: %s", e.what());
        return false;
    }
}

void VESC::disconnect() {
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    running = false;
    if (!serial_port_) return;
    
        try {
        serial_port_->Close();
        if (!VESC::getSuppressLogs()) RCLCPP_INFO(logger, "Disconnected.");
    } catch(...) {
        if (!VESC::getSuppressLogs()) RCLCPP_WARN(logger, "Close failed during disconnect (USB gone?)");
    }
}

bool VESC::isConnected(){
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    if (!running || !serial_port_)
        return false;
    try{
        return serial_port_->IsOpen();
    }catch(const std::exception& e){
        if (!VESC::getSuppressLogs()) RCLCPP_DEBUG(logger, "IsOpen() threw exception: %s", e.what());
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

void VESC::setId(uint8_t id) {
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    motor_id = id;
}

static void safe_reset_port(std::unique_ptr<LibSerial::SerialPort>& port) {
    if (!port) return;
    try { port->Close(); } catch (...) {}
    try { port.reset(); } catch (...) { port.release(); }
}

bool VESC::tryConnectToPort(const std::string& port) {
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    try {
        if (serial_port_ && serial_port_->IsOpen()) serial_port_->Close();
        safe_reset_port(serial_port_);
        serial_port_ = std::make_unique<SerialPort>();
        serial_port_->Open(port);
        setupPort();
        try {
            serial_port_->FlushInputBuffer();
            serial_port_->FlushIOBuffers();
            } catch (const std::exception& e) {
            if (!VESC::getSuppressLogs()) RCLCPP_WARN(logger, "Flush failed on %s: %s", port.c_str(), e.what());
            safe_reset_port(serial_port_);
            return false;
        }
        running = true;
        VESCData data;
        for (int attempt = 0; attempt < 3; ++attempt) {
            if (attempt > 0) { try { serial_port_->FlushInputBuffer(); } catch(...) {} }
            if (get_telemetry(data) && data.motor_controller_id == motor_id) {
                port_name = port;
                if (!VESC::getSuppressLogs()) RCLCPP_INFO(logger, "Motor ID=%u connected to %s", motor_id, port.c_str());
                return true;
            }
        }
        running = false;
        safe_reset_port(serial_port_);
        return false;
    } catch (const std::exception& e) {
        if (!VESC::getSuppressLogs()) RCLCPP_WARN(logger, "Port %s failed for motor_id=%u: %s", port.c_str(), motor_id, e.what());
        running = false;
        safe_reset_port(serial_port_);
        return false;
    } catch (...) {
        if (!VESC::getSuppressLogs()) RCLCPP_WARN(logger, "Port %s failed for motor_id=%u: unknown error", port.c_str(), motor_id);
        running = false;
        safe_reset_port(serial_port_);
        return false;
    }
}

bool VESC::autoConnect(const std::string& hint) {
    try {
        // Fast path: try the cached port directly — no global scan lock needed
        if (!hint.empty()) {
            if (!VESC::getSuppressLogs()) RCLCPP_INFO(logger, "Motor ID=%u trying hint port %s", motor_id, hint.c_str());
            if (tryConnectToPort(hint)) return true;
            if (!VESC::getSuppressLogs()) RCLCPP_WARN(logger, "Hint port %s failed for motor_id=%u, falling back to scan", hint.c_str(), motor_id);
        }

        // Slow path: scan all ports under global lock to avoid two motors racing on the same port
        std::lock_guard<std::mutex> scan_lk(scan_mutex_);
        auto ports = scanPorts();
        if (!VESC::getSuppressLogs()) RCLCPP_INFO(logger, "Scanning for motor_id=%u across %zu ports", motor_id, ports.size());
        for (const auto& port : ports) {
            if (port == hint) continue;  // already tried above
            if (tryConnectToPort(port)) return true;
        }
        if (!VESC::getSuppressLogs()) RCLCPP_ERROR(logger, "Failed to find motor_id=%u after scanning all ports", motor_id);
        return false;
    } catch (const std::exception& e) {
        if (!VESC::getSuppressLogs()) RCLCPP_ERROR(logger, "autoConnect exception for motor_id=%u: %s", motor_id, e.what());
        running = false;
        return false;
    } catch (...) {
        if (!VESC::getSuppressLogs()) RCLCPP_ERROR(logger, "autoConnect unknown exception for motor_id=%u", motor_id);
        running = false;
        return false;
    }
}

void VESC::set_rpm(int32_t rpm) {
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
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
        if (!VESC::getSuppressLogs()) RCLCPP_ERROR(logger, "set_rpm unknown error");
        running = false;
        try { if(serial_port_) serial_port_->Close(); } catch(...) {}
    }
}

void VESC::set_duty_cycle(float duty) {
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    if (!isConnected()) return;

    try {
        int32_t scaled = static_cast<int32_t>(duty * 100000.0f);
        std::vector<uint8_t> payload;
        payload.push_back(5); // COMM_SET_DUTY
        payload.push_back(static_cast<uint8_t>((scaled >> 24) & 0xFF));
        payload.push_back(static_cast<uint8_t>((scaled >> 16) & 0xFF));
        payload.push_back(static_cast<uint8_t>((scaled >>  8) & 0xFF));
        payload.push_back(static_cast<uint8_t>( scaled        & 0xFF));
        send_vesc_packet(payload);
    } catch (...) {
        if (!VESC::getSuppressLogs()) RCLCPP_ERROR(logger, "set_duty_cycle unknown error");
        running = false;
        try { if (serial_port_) serial_port_->Close(); } catch (...) {}
    }
}

void VESC::send_vesc_packet(const std::vector<uint8_t> &payload) {
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
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
        if (!VESC::getSuppressLogs()) RCLCPP_WARN(logger, "Write failed (unknown error)");
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
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    if (!isConnected()) return;

    std::vector<uint8_t> payload;
    payload.push_back(4); 
    send_vesc_packet(payload);
}

std::vector<uint8_t> VESC::read_bytes() {
    std::vector<uint8_t> buffer;
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    if (!isConnected()) return buffer;

    try {
        char byte;
        int junk_count = 0;
        
        // Scan forward until the VESC frame start byte (0x02).
        // A single non-0x02 byte used to abort the whole read; now we skip
        // junk bytes so one bad byte doesn't permanently desync telemetry.
        do {
            serial_port_->ReadByte(byte, timeout);
            junk_count++;
            
            // Safety break out to prevent infinite loop on broken file descriptor
            if (junk_count > 100) {
                running = false;
                if (!VESC::getSuppressLogs()) {
                    RCLCPP_WARN(logger, "Read limit exceeded without finding 0x02 frame start.");
                }
                return buffer; 
            }
        } while (static_cast<uint8_t>(byte) != 0x02);
        
        buffer.push_back(static_cast<uint8_t>(byte));
                                                                                                                                                                                                        
        // Read length                                                                                                                                                                                  
        serial_port_->ReadByte(byte, timeout);                                                                                                                                                            
        uint8_t length = static_cast<uint8_t>(byte);                                                                                                                                                    
        buffer.push_back(length);                                                                                                                                                                         
 
        // Read payload + 2 CRC bytes + end byte                                                                                                                                                          
        for (int i = 0; i < length + 3; i++) {                                                                                                                                                          
            serial_port_->ReadByte(byte, timeout);                                                                                                                                                        
            buffer.push_back(static_cast<uint8_t>(byte));
        }                                                                                                                                                                                                 
    } catch (const ReadTimeout&) {                                                                                                                                                                        
    } catch (...) {
        running = false;                                                                                                                                                                                  
    }                                                                                                                                                                                                   
    return buffer;                                                                                                                                                                                        
}

bool VESC::get_telemetry(VESCData& out) {
    std::lock_guard<std::recursive_mutex> lk(port_mutex_);
    if (!isConnected()) return false;
    // Flush stale bytes before requesting — prevents cascading desync when a
    // previous read timed out and left partial VESC response bytes in the OS buffer.
    try { serial_port_->FlushInputBuffer(); } catch (...) {}
    request_values();
    if (!running) return false;

    auto raw = read_bytes();
        if (raw.empty()) {
        if (!VESC::getSuppressLogs()) RCLCPP_WARN(logger, "read_bytes() returned empty — no data from VESC (timeout)");
        return false;
    }

    auto payload = find_packet(raw);
    if (payload.empty()) {
        if (!VESC::getSuppressLogs()) RCLCPP_WARN(logger, "find_packet() failed: got %zu raw bytes but no valid VESC frame", raw.size());
        return false;
    }

    if (payload.size() < 29 || payload[0] != 4) {
        if (!VESC::getSuppressLogs()) RCLCPP_WARN(logger, "Invalid payload: size=%zu, cmd=0x%02X (expected >=29 bytes, cmd=0x04)",
            payload.size(), payload.empty() ? 0 : payload[0]);
        return false;
    }

    // Offsets from official VESC firmware
    auto get_i16 = [&](int i) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(payload[i]) << 8) | payload[i+1]);
    };

    auto get_i32 = [&](int i) -> int32_t {
        return static_cast<int32_t>(
            (static_cast<uint32_t>(payload[i])   << 24) |
            (static_cast<uint32_t>(payload[i+1]) << 16) |
            (static_cast<uint32_t>(payload[i+2]) <<  8) |
             static_cast<uint32_t>(payload[i+3])
        );
    };
    // Payload layout (COMM_GET_VALUES = 0x04):
    //  [1-2]   temp_fet        i16 /10
    //  [5-8]   current_motor   i32 /100
    //  [9-12]  current_in      i32 /100
    //  [21-22] duty_cycle      i16 /1000
    //  [23-26] rpm             i32
    //  [27-28] input_voltage   i16 /10
    //  [53]    fault_code      u8
    //  [54-57] position        i32 /1000000  (degrees)
    //  [58]    motor_id        u8
    out.temp_fet      = get_i16(1)  / 10.0f;
    out.current_motor = get_i32(5)  / 100.0f;
    out.current_in    = get_i32(9)  / 100.0f;
    out.duty_cycle    = get_i16(21) / 1000.0f;
    out.rpm           = get_i32(23);
    out.input_voltage = get_i16(27) / 10.0f;

    if (payload.size() > 53) out.fault_code = payload[53];
    if (payload.size() > 57) out.position   = get_i32(54) / 1000000.0f;
    if (payload.size() > 58) out.motor_controller_id = payload[58];
    return true;
}
