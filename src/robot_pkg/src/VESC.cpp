#include "robot_pkg/VESC.hpp"
#include <algorithm>
#include <cstring>
#include <filesystem>

#define rosInfo(fmt, ...)  RCLCPP_INFO(logger, fmt, ##__VA_ARGS__)
#define rosError(fmt, ...) RCLCPP_ERROR(logger, fmt, ##__VA_ARGS__)
#define rosDebug(fmt, ...) RCLCPP_DEBUG(rclcpp::get_logger("VESC"), fmt, ##__VA_ARGS__)

using namespace LibSerial;
using namespace std;

VESC::VESC(uint8_t id, int baud, int to) : motor_id(id), baudrate(baud), timeout(to),logger(rclcpp::get_logger("VESC")) {
        serial_port_ = std::make_unique<SerialPort>();
      }

VESC::~VESC() {
    disconnect();
}

bool VESC::connect() {
    try {
        // 1. Open the port
        serial_port_->Open(port_name);

        // 2. Configure Baud Rate (LibSerial requires specific enums)
        // You might need a switch case if you want to support multiple rates dynamically
        if (baudrate == 115200) {
            serial_port_->SetBaudRate(BaudRate::BAUD_115200);
        } else {
            serial_port_->SetBaudRate(BaudRate::BAUD_9600); // Default fallback
        }

        // 3. Configure 8N1 (Standard for VESC)
        serial_port_->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
        serial_port_->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
        serial_port_->SetParity(Parity::PARITY_NONE);
        serial_port_->SetStopBits(StopBits::STOP_BITS_1);

        running = true;
        RCLCPP_INFO(logger, "Connected to %s", port_name.c_str());
        return true;
    } 
    catch (const OpenFailed&) {
        RCLCPP_ERROR(logger, "Failed to open port %s", port_name.c_str());
        return false;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Serial Exception: %s", e.what());
        return false;
    }
}

void VESC::disconnect() {
    if (serial_port_->IsOpen()) {
        serial_port_->Close();
        RCLCPP_INFO(logger, "Disconnected.");
    }
    running = false;
}

bool VESC::isConnected()
{
    if (!running)
        return false;

    try{
        if(!serial_port_->IsOpen()){
            return false;
        }
    }catch(...){
            return false;
        }
    
    
    return running;
}

vector<string> scanPorts(){
    vector<string> ports;

    for(auto const& entry : filesystem::directory_iterator("/dev")){
        string name = entry.path().string();

        if(name.find("ttyACM") != string::npos){
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
            if(serial_port_->IsOpen()){
                try{serial_port_->Close();} catch(...){}
                
            }

            if(!serial_port_->IsOpen()){
            serial_port_->Open(port);

            if (baudrate == 115200) {
                serial_port_->SetBaudRate(BaudRate::BAUD_115200);
            } else {
                serial_port_->SetBaudRate(BaudRate::BAUD_9600); // Default fallback
            }

            // 3. Configure 8N1 (Standard for VESC)
            serial_port_->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            serial_port_->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            serial_port_->SetParity(Parity::PARITY_NONE);
            serial_port_->SetStopBits(StopBits::STOP_BITS_1);

            // ... (your 8N1 configuration code) ...
            serial_port_->SetStopBits(StopBits::STOP_BITS_1);
            
            // --- ADD THIS STABILIZATION BLOCK ---
            RCLCPP_INFO(logger, "Settling port %s...", port.c_str());
            
            // 1. Give the VESC and the USB driver time to "sync up"
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); 

            // 2. Clear any "garbage" bytes that were in the buffer before we started
            if (serial_port_->IsDataAvailable()) {
                serial_port_->FlushInputBuffer();
                serial_port_->FlushIOBuffers();
                serial_port_->FlushOutputBuffer();
                RCLCPP_INFO(logger, "Buffers flushed.");
            }
            }
            RCLCPP_INFO(logger,"tried");


        
            VESCData data;
            running = true;

            if(get_telemetry(data)){
                    RCLCPP_INFO(logger,"tried2");

                    RCLCPP_INFO(logger,
                    "Attempt connection  to ID %f ",
                    data.motor_controller_id);


                if(data.motor_controller_id == motor_id){
                    RCLCPP_INFO(logger,
                    "Auto connected to ID %d on %s",
                    motor_id,port.c_str());
                    port_name = port;
                    return true;
                }
            }

            serial_port_->Close();

        }
        catch(const exception& e){
            RCLCPP_INFO(logger,"POrt failed brcause: %s",e.what());

            running = false;
            try{serial_port_->Close();} catch(...){}
            continue;
        }
    }
        return false;
}

void VESC::set_rpm(int32_t rpm) {
    if (!running || !serial_port_->IsOpen()) return;

    try{
    std::vector<uint8_t> payload;
    payload.push_back(8); // COMM_SET_RPM
    payload.push_back((rpm >> 24) & 0xFF);
    payload.push_back((rpm >> 16) & 0xFF);
    payload.push_back((rpm >> 8) & 0xFF);
    payload.push_back(rpm & 0xFF);

    send_vesc_packet(payload);
    }catch(const exception& e){
        running = false;
            try { serial_port_->Close(); } catch(...) {}

    }
}

void VESC::send_vesc_packet(const std::vector<uint8_t> &payload) {
    if (!running || !serial_port_->IsOpen()) return;
    try{
    std::vector<uint8_t> packet;
    packet.push_back(2); // Start
    packet.push_back(static_cast<uint8_t>(payload.size()));
    packet.insert(packet.end(), payload.begin(), payload.end());

    uint16_t crc = crc16(payload);
    packet.push_back((crc >> 8) & 0xFF);
    packet.push_back(crc & 0xFF);
    packet.push_back(3); // End

    serial_port_->Write(packet);
    }catch(const exception& e){
        running = false;
            try { serial_port_->Close(); } catch(...) {}

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
        rosDebug("Respuesta demasiado corta o vacía: %zu bytes", response.size());
        return {};
    }
    
    for (size_t i = 0; i < response.size() - 2; i++) {
        if (response[i] == 2) {  // Start byte
            uint8_t length = response[i + 1];
            size_t packet_end = i + 2 + length + 3;
            
            if (packet_end <= response.size() && response[packet_end - 1] == 3) {  // End byte
                // Extract data
                std::vector<uint8_t> data(response.begin() + i + 2, 
                                         response.begin() + i + 2 + length);
                
                // Extract CRC
                uint16_t crc_received = (response[i + 2 + length] << 8) | 
                                       response[i + 2 + length + 1];
                uint16_t crc_calculated = crc16(data);
                
                if (crc_received == crc_calculated) {
                    return data;
                }
            }
        }
    }
    
    rosDebug("No se encontró un paquete válido en la respuesta.");
    return {};
}

void VESC::request_values() {
    if (!running || !serial_port_->IsOpen()) return;

    std::vector<uint8_t> payload;
    payload.push_back(4); // COMM_GET_VALUES
    send_vesc_packet(payload);
}

std::vector<uint8_t> VESC::read_bytes() {
    std::vector<uint8_t> buffer;

    try{
    while (serial_port_->IsDataAvailable()) {
        char byte;
        serial_port_->ReadByte(byte, timeout);
        buffer.push_back(static_cast<uint8_t>(byte));
    }
}catch(const exception& e){
    running =false;
        try { serial_port_->Close(); } catch(...) {}

}

    return buffer;
}   

bool VESC::get_telemetry(VESCData& out) {
    request_values();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    auto raw = read_bytes();
    auto payload = find_packet(raw);

    if (payload.empty()) return false;

    if (payload[0] != 4) return false; // not GET_VALUES

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
RCLCPP_INFO(logger,"triedTElemtry");
    out.temp_fet      = get_i16(1) / 10.0f;
    out.current_motor = get_i32(5) / 100.0f;
    out.rpm           = get_i32(23);
    out.input_voltage = get_i16(29) / 10.0f;

    if(payload.size() > 58){
        out.motor_controller_id = payload[58];
    }
    return true;
}

float VESC::current_motor(const std::vector<uint8_t>& data){
    if (data.size() < 9) return 0.0f;
    
    // Convert big-endian int32 at offset 5-9
    int32_t value = (data[5] << 24) | (data[6] << 16) | (data[7] << 8) | data[8];
    return value / 100.0f;
}

float VESC::temp_mos1(const std::vector<uint8_t>& data){
    if (data.size() < 61) return 0.0f;
    
    // Convert big-endian int16 at offset 59-61
    int16_t value = (data[59] << 8) | data[60];
    return value / 10.0f;
}

float VESC::temp_motor(const std::vector<uint8_t>& /* data */){
    // Motor temperature not available in this structure
    return 0.0f;
}