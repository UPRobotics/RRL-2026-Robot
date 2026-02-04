#include "robot_pkg/VESC.hpp"
#include <algorithm>
#include <cstring>
#include <chrono>
#include <thread>

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

std::vector<uint8_t> VESC::build_packet(const std::vector<uint8_t>& payload){
    std::vector<uint8_t> packet;
    uint8_t length = payload.size();
    
    // Header: start byte (2) y longitud
    packet.push_back(2);
    packet.push_back(length);
    
    // Payload
    packet.insert(packet.end(), payload.begin(), payload.end());
    
    // CRC
    uint16_t crc = crc16(payload);
    packet.push_back((crc >> 8) & 0xFF);
    packet.push_back(crc & 0xFF);
    
    // End byte
    packet.push_back(3);
    
    return packet;
}

void VESC::send_rpm(int32_t rpm){
    if (!serial_connection || !serial_connection->isOpen()) {
        rosError("No hay conexión activa en %s Motor id: %d", 
                 serial_port.c_str(), motor_id);
        return;
    }
    
    try {
        std::vector<uint8_t> payload;
        payload.push_back(8);  // COMM_SET_RPM
        
        // Convertir RPM a big-endian int32
        payload.push_back((rpm >> 24) & 0xFF);
        payload.push_back((rpm >> 16) & 0xFF);
        payload.push_back((rpm >> 8) & 0xFF);
        payload.push_back(rpm & 0xFF);
        
        std::vector<uint8_t> packet = build_packet(payload);
        serial_connection->write(packet);
        
        // Pequeño delay para estabilidad
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    catch (serial::IOException& e) {
        rosError("Error al enviar RPM a %s Motor id: %d - %s", 
                 serial_port.c_str(), motor_id, e.what());
        
        // Intentar reconectar
        try {
            if (serial_connection) {
                serial_connection->close();
            }
            serial_connection.reset();
            rosInfo("Intentando reconectar a %s Motor id: %d", 
                    serial_port.c_str(), motor_id);
            connect();
        }
        catch (...) {
            // Ignorar errores de reconexión
        }
    }
}

void VESC::send_duty_cycle(float duty_cycle){
    if (!serial_connection || !serial_connection->isOpen()) {
        rosError("No hay conexión activa en %s Motor id: %d", 
                 serial_port.c_str(), motor_id);
        return;
    }
    
    try {
        // Duty cycle debe estar entre -1.0 y 1.0, se convierte a entero * 100000
        int32_t duty_int = static_cast<int32_t>(duty_cycle * 100000);
        
        std::vector<uint8_t> payload;
        payload.push_back(5);  // COMM_SET_DUTY
        
        // Convertir duty_int a big-endian int32
        payload.push_back((duty_int >> 24) & 0xFF);
        payload.push_back((duty_int >> 16) & 0xFF);
        payload.push_back((duty_int >> 8) & 0xFF);
        payload.push_back(duty_int & 0xFF);
        
        std::vector<uint8_t> packet = build_packet(payload);
        serial_connection->write(packet);
        
        // Pequeño delay para estabilidad
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    catch (serial::IOException& e) {
        rosError("Error al enviar duty cycle a %s Motor id: %d - %s", 
                 serial_port.c_str(), motor_id, e.what());
        
        // Intentar reconectar
        try {
            if (serial_connection) {
                serial_connection->close();
            }
            serial_connection.reset();
            rosInfo("Intentando reconectar a %s Motor id: %d", 
                    serial_port.c_str(), motor_id);
            connect();
        }
        catch (...) {
            // Ignorar errores de reconexión
        }
    }
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

VESCData VESC::get_values(std::mutex& data_lock, std::map<uint8_t, VESCData>& vesc_data_map){
    VESCData vesc_data;
    
    if (!serial_connection || !serial_connection->isOpen()) {
        rosDebug("No hay conexión activa en %s Motor id: %d", 
                 serial_port.c_str(), motor_id);
        return vesc_data;
    }
    
    try {
        // Crear paquete COMM_GET_VALUES
        std::vector<uint8_t> payload = {4};  // COMM_GET_VALUES
        std::vector<uint8_t> packet = build_packet(payload);
        serial_connection->write(packet);
        
        // Limpiar buffer y esperar respuesta
        serial_connection->flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Leer respuesta
        std::vector<uint8_t> response;
        if (serial_connection->available() > 0) {
            size_t bytes_to_read = std::min(static_cast<size_t>(100), 
                                           serial_connection->available());
            response.resize(bytes_to_read);
            serial_connection->read(response.data(), bytes_to_read);
        }
        
        if (response.empty()) {
            rosDebug("No se recibió respuesta de %s Motor id: %d", 
                     serial_port.c_str(), motor_id);
            return vesc_data;
        }
        
        // Buscar y extraer paquete válido
        std::vector<uint8_t> data = find_packet(response);
        if (data.empty()) {
            rosDebug("No se extrajeron datos válidos de %s Motor id: %d", 
                     serial_port.c_str(), motor_id);
            return vesc_data;
        }
        
        // Parsear datos según protocolo VESC oficial
        size_t offset = 1;  // Saltar el byte de comando
        
        // Mask bit 0: temp_fet - int16 con escala 1e1
        if (data.size() >= offset + 2) {
            int16_t temp_fet = (data[offset] << 8) | data[offset + 1];
            vesc_data.temp_fet = temp_fet / 10.0f;
            offset += 2;
        }
        
        // Mask bit 1: temp_motor - int16 con escala 1e1
        if (data.size() >= offset + 2) {
            int16_t temp_motor = (data[offset] << 8) | data[offset + 1];
            vesc_data.temp_motor = temp_motor / 10.0f;
            offset += 2;
        }
        
        // Mask bit 2: current_motor (avg_motor_current) - int32 con escala 1e2
        if (data.size() >= offset + 4) {
            int32_t current_motor = (data[offset] << 24) | (data[offset + 1] << 16) | 
                                   (data[offset + 2] << 8) | data[offset + 3];
            vesc_data.current_motor = current_motor / 100.0f;
            offset += 4;
        }
        
        // Mask bit 3: current_in (avg_input_current) - int32 con escala 1e2
        if (data.size() >= offset + 4) {
            int32_t current_in = (data[offset] << 24) | (data[offset + 1] << 16) | 
                                (data[offset + 2] << 8) | data[offset + 3];
            vesc_data.current_in = current_in / 100.0f;
            offset += 4;
        }
        
        // Mask bit 4: id (avg_id) - int32 con escala 1e2
        if (data.size() >= offset + 4) {
            int32_t current_id = (data[offset] << 24) | (data[offset + 1] << 16) | 
                                (data[offset + 2] << 8) | data[offset + 3];
            vesc_data.current_id = current_id / 100.0f;
            offset += 4;
        }
        
        // Mask bit 5: iq (avg_iq) - int32 con escala 1e2
        if (data.size() >= offset + 4) {
            int32_t current_iq = (data[offset] << 24) | (data[offset + 1] << 16) | 
                                (data[offset + 2] << 8) | data[offset + 3];
            vesc_data.current_iq = current_iq / 100.0f;
            offset += 4;
        }
        
        // Mask bit 6: duty_now - int16 con escala 1e3
        if (data.size() >= offset + 2) {
            int16_t duty_cycle = (data[offset] << 8) | data[offset + 1];
            vesc_data.duty_cycle = duty_cycle / 1000.0f;
            offset += 2;
        }
        
        // Mask bit 7: rpm - int32 sin escala
        if (data.size() >= offset + 4) {
            vesc_data.rpm = (data[offset] << 24) | (data[offset + 1] << 16) | 
                           (data[offset + 2] << 8) | data[offset + 3];
            offset += 4;
        }
        
        // Mask bit 8: v_in (input_voltage) - int16 con escala 1e1
        if (data.size() >= offset + 2) {
            int16_t input_voltage = (data[offset] << 8) | data[offset + 1];
            vesc_data.input_voltage = input_voltage / 10.0f;
            offset += 2;
        }
        
        // Mask bit 9: amp_hours - int32 con escala 1e4
        if (data.size() >= offset + 4) {
            int32_t amp_hours = (data[offset] << 24) | (data[offset + 1] << 16) | 
                               (data[offset + 2] << 8) | data[offset + 3];
            vesc_data.amp_hours = amp_hours / 10000.0f;
            offset += 4;
        }
        
        // Mask bit 10: amp_hours_charged - int32 con escala 1e4
        if (data.size() >= offset + 4) {
            int32_t amp_hours_charged = (data[offset] << 24) | (data[offset + 1] << 16) | 
                                       (data[offset + 2] << 8) | data[offset + 3];
            vesc_data.amp_hours_charged = amp_hours_charged / 10000.0f;
            offset += 4;
        }
        
        // Mask bit 11: watt_hours - int32 con escala 1e4
        if (data.size() >= offset + 4) {
            int32_t watt_hours = (data[offset] << 24) | (data[offset + 1] << 16) | 
                                (data[offset + 2] << 8) | data[offset + 3];
            vesc_data.watt_hours = watt_hours / 10000.0f;
            offset += 4;
        }
        
        // Mask bit 12: watt_hours_charged - int32 con escala 1e4
        if (data.size() >= offset + 4) {
            int32_t watt_hours_charged = (data[offset] << 24) | (data[offset + 1] << 16) | 
                                        (data[offset + 2] << 8) | data[offset + 3];
            vesc_data.watt_hours_charged = watt_hours_charged / 10000.0f;
            offset += 4;
        }
        
        // Mask bit 13: tachometer - int32 sin escala
        if (data.size() >= offset + 4) {
            vesc_data.tachometer = (data[offset] << 24) | (data[offset + 1] << 16) | 
                                  (data[offset + 2] << 8) | data[offset + 3];
            offset += 4;
        }
        
        // Mask bit 14: tachometer_abs - int32 sin escala
        if (data.size() >= offset + 4) {
            vesc_data.tachometer_abs = (data[offset] << 24) | (data[offset + 1] << 16) | 
                                      (data[offset + 2] << 8) | data[offset + 3];
            offset += 4;
        }
        
        // Mask bit 15: fault_code - uint8
        if (data.size() >= offset + 1) {
            vesc_data.fault_code = data[offset];
            offset += 1;
        }
        
        // Guardar datos crudos para depuración
        vesc_data.raw_data = data;
        
        // Log de depuración
        rosDebug("Motor %d telemetry: Voltage=%.1fV, Current=%.2fA, RPM=%d, "
                 "FET_Temp=%.1f°C, Motor_Temp=%.1f°C, Duty=%.3f, Fault=%d",
                 motor_id, vesc_data.input_voltage, vesc_data.current_motor, 
                 vesc_data.rpm, vesc_data.temp_fet, vesc_data.temp_motor,
                 vesc_data.duty_cycle, vesc_data.fault_code);
        
        // Actualizar mapa compartido
        {
            std::lock_guard<std::mutex> lock(data_lock);
            vesc_data_map[motor_id] = vesc_data;
        }
        
        return vesc_data;
    }
    catch (const std::exception& e) {
        rosError("Error al obtener valores de %s Motor id: %d - %s", 
                 serial_port.c_str(), motor_id, e.what());
        return vesc_data;
    }
}