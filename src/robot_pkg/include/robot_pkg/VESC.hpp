#ifndef VESC_HPP
#define VESC_HPP

#include <string>
#include <memory>
#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <cstdint>

class VESC{
    private:
        std::string serial_port;
        uint8_t motor_id;
        int baudrate;
        int timeout;
        std::unique_ptr<serial::Serial> serial_connection;
        bool running = false;
        rclcpp::Logger logger;
        
    public:
        VESC(std::string port, uint8_t id, int baud = 115200, int to = 1000);
        ~VESC();

        bool connect();
        void disconnect();
        
        // Métodos estáticos
        static uint16_t crc16(const std::vector<uint8_t>& data, uint16_t poly = 0x1021, uint16_t init_val = 0);
        static std::vector<uint8_t> find_packet(const std::vector<uint8_t>& response);
        static float current_motor(const std::vector<uint8_t>& data);
        static float temp_mos1(const std::vector<uint8_t>& data);
        static float temp_motor(const std::vector<uint8_t>& data);
};

#endif // VESC_HPP