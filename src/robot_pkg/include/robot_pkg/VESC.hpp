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
#include <atomic>

struct VESCData {
    float temp_fet = 0.0f;
    float temp_motor = 0.0f;
    float current_motor = 0.0f;
    float current_in = 0.0f;
    uint8_t motor_controller_id = 0xFF;
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
    float   position   = 0.0f;   // degrees
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
        std::recursive_mutex port_mutex_;
        rclcpp::Logger logger;

        // Shared across all VESC instances — prevents two motors from scanning
        // the same port simultaneously during autoConnect()
        static std::mutex scan_mutex_;
        static std::atomic<bool> s_suppress_logs_;
        
        void setupPort();
        // Try to connect to one specific port and verify motor ID. No scan_mutex_ acquired.
        bool tryConnectToPort(const std::string& port);

        // Métodos estáticos
        static uint16_t crc16(const std::vector<uint8_t>& data, uint16_t poly = 0x1021, uint16_t init_val = 0);
        static std::vector<uint8_t> find_packet(const std::vector<uint8_t>& response);
        
    public:
        VESC(uint8_t id, int baud = 115200, int to = 1000);
        ~VESC();

        bool connect();
        void disconnect();
        // hint: try this port first (no global scan lock); falls back to full scan if empty or fails
        bool autoConnect(const std::string& hint = "");
        bool isConnected();
        void setId(uint8_t id);
        std::string getPortName() const { return port_name; }
        
        // Write data to the VESC
        void send_vesc_packet(const std::vector<uint8_t> &payload);
        void set_rpm(int32_t rpm);
        void set_duty_cycle(float duty);   // duty in [-1.0, 1.0]
        void request_values();
        void set_position(float degrees);
        std::vector<uint8_t> read_bytes();
        bool get_telemetry(VESCData &out);

        // Control logging from external code: when true, VESC logging is suppressed.
        static void setSuppressLogs(bool v) { s_suppress_logs_.store(v); }
        static bool getSuppressLogs() { return s_suppress_logs_.load(); }

};
#endif // VESC_HPP