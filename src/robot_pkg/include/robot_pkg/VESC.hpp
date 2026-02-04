#ifndef VESC_HPP
#define VESC_HPP

#include <string>
#include <memory>
#include <serial/serial.h>
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
        std::string serial_port;
        uint8_t motor_id;
        int baudrate;
        int timeout;
        std::unique_ptr<serial::Serial> serial_connection;
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
        
        /*
        Los metodos de control son los únicos que se usarán externamente.        
        */
        
        // Métodos de control
        void send_rpm(int32_t rpm);
        void send_duty_cycle(float duty_cycle);
        VESCData get_values(std::mutex& data_lock, std::map<uint8_t, VESCData>& vesc_data_map);
};
#endif // VESC_HPP