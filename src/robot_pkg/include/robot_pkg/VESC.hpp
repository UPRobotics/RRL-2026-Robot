#ifndef VESC_HPP
#define VESC_HPP

#include <string>
#include <memory>
#include <libserial/SerialPort.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cstdint>
#include <map>
#include <mutex>

struct VESCData {
    float temp_fet = 0.0f;
    float temp_motor = 0.0f;
    float current_motor = 0.0f;
    float current_in = 0.0f;
    float current_id = 0.0f;
    float current_iq = 0.0f;
    float duty_cycle = 0.0f;
    int32_t rpm = 0;
    float input_voltage = 0.0f;
    float amp_hours = 0.0f;
    float amp_hours_charged = 0.0f;
    float watt_hours = 0.0f;
    float watt_hours_charged = 0.0f;
    int32_t tachometer = 0;
    int32_t tachometer_abs = 0;
    uint8_t fault_code = 0;
    std::vector<uint8_t> raw_data;
};

class VESC{
    private:
        std::string port_name;
        uint8_t motor_id;
        int baudrate;
        int timeout;
        std::unique_ptr<LibSerial::SerialPort> serial_port_;
        bool running = false;
        rclcpp::Logger logger;
        
        // Métodos estáticos privados
        static uint16_t crc16(const std::vector<uint8_t>& data, uint16_t poly = 0x1021, uint16_t init_val = 0);
        static std::vector<uint8_t> build_packet(const std::vector<uint8_t>& payload);
        static std::vector<uint8_t> find_packet(const std::vector<uint8_t>& response);
        static float current_motor(const std::vector<uint8_t>& data);
        static float temp_mos1(const std::vector<uint8_t>& data);
        static float temp_motor(const std::vector<uint8_t>& data);
        
    public:
        VESC(std::string port, uint8_t id, int baud = 115200, int to = 1000);
        ~VESC();

        bool connect();
        void disconnect();
        
        // Write data to the VESC
        void send_vesc_packet(const std::vector<uint8_t> &payload);
        void set_rpm(int32_t rpm);
        // Métodos estáticos
        static uint16_t crc16(const std::vector<uint8_t>& data, uint16_t poly = 0x1021, uint16_t init_val = 0);
        static std::vector<uint8_t> find_packet(const std::vector<uint8_t>& response);
        static float current_motor(const std::vector<uint8_t>& data);
        static float temp_mos1(const std::vector<uint8_t>& data);
        static float temp_motor(const std::vector<uint8_t>& data);
};
#endif // VESC_HPP