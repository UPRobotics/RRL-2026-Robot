#include "robot_pkg/VESC.hpp"
#include <algorithm>
#include <cstring>

#define rosInfo(fmt, ...)  RCLCPP_INFO(logger, fmt, ##__VA_ARGS__)
#define rosError(fmt, ...) RCLCPP_ERROR(logger, fmt, ##__VA_ARGS__)
#define rosDebug(fmt, ...) RCLCPP_DEBUG(rclcpp::get_logger("VESC"), fmt, ##__VA_ARGS__)

VESC::VESC(std::string port, uint8_t id, int baud, int to)
    : serial_port(port), motor_id(id), baudrate(baud), timeout(to),
      logger(rclcpp::get_logger("VESC")) {}

VESC::~VESC() {
    disconnect();
}

bool VESC::connect(){
    try{
        serial_connection = std::make_unique<serial::Serial>(
            serial_port, baudrate, serial::Timeout::simpleTimeout(timeout));
        running = true;
        rosInfo("Conectado correctamente al %s Motor id: %d", 
                serial_port.c_str(), motor_id);
        return true;
    }
    catch (serial::IOException& e){
        rosError("Error al conectar al %s Motor id: %d", 
                 serial_port.c_str(), motor_id);
        serial_connection.reset();
        running = false;
        return false;
    }
}

void VESC::disconnect(){
    if(serial_connection && running){
        try{
            serial_connection->close();
            rosInfo("Desconectado de %s Motor id: %d", 
                    serial_port.c_str(), motor_id);
        }
        catch(serial::IOException& e){
            rosError("Error al desconectar %s Motor id: %d", 
                     serial_port.c_str(), motor_id);
        }
        serial_connection.reset();
        running = false;
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

float VESC::temp_motor(const std::vector<uint8_t>& data){
    // Motor temperature not available in this structure
    return 0.0f;
}