
import serial
import struct
import time
import threading
import math
import signal
import sys
from collections import defaultdict

# SERIAL PORTS CONFIGURATION - Edit this section to add/remove motors
SERIAL_PORTS = {
    "/dev/ttyACM0": {
        "direction": 1, 
        "target_rpm": 1000,
        "control_mode": "duty_cycle",  # "rpm" o "duty_cycle"
        "target_duty_cycle": 0.1  # Solo usado si control_mode es "duty_cycle", valor entre -1.0 y 1.0
    },
    # Ejemplo con duty cycle:
    # "/dev/ttyACM6": {
    #     "direction": 1, 
    #     "target_rpm": None,  # No usado en modo duty_cycle
    #     "control_mode": "duty_cycle",
    #     "target_duty_cycle": 0.3  # 30% duty cycle
    # },
    # Agrega más puertos según sea necesario
}
BAUDRATE = 115200


class VESCController:
    def __init__(self, serial_port, motor_id, baudrate=115200, timeout=0.1):
        self.serial_port = serial_port
        self.motor_id = motor_id  # Identificador del motor (e.g., "Motor 1")
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False

    def connect(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=self.timeout)
            print(f"Conectado a {self.serial_port} ({self.motor_id})")
            return True
        except serial.SerialException as e:
            print(f"Error al conectar a {self.serial_port} ({self.motor_id}): {e}")
            self.ser = None
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print(f"Desconectado de {self.serial_port} ({self.motor_id})")
            except serial.SerialException as e:
                print(f"Error al desconectar de {self.serial_port} ({self.motor_id}): {e}")
            finally:
                self.ser = None

    @staticmethod
    def crc16(data, poly=0x1021, init_val=0):
        crc = init_val
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ poly) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    @staticmethod
    def build_packet(payload):
        length = len(payload)
        header = bytearray([2, length])
        crc = VESCController.crc16(payload)
        packet = header + payload + bytearray([(crc >> 8) & 0xFF, crc & 0xFF, 3])
        return bytes(packet)

    def send_rpm(self, rpm):
        if not self.ser or not self.ser.is_open:
            print(f"No hay conexión activa en {self.serial_port} ({self.motor_id})")
            return
        try:
            payload = bytearray([8])  # COMM_SET_RPM
            payload.extend(struct.pack(">i", rpm))
            packet = self.build_packet(payload)
            self.ser.write(packet)
        except serial.SerialException as e:
            print(f"Error al enviar RPM a {self.serial_port} ({self.motor_id}): {e}")

    def send_duty_cycle(self, duty_cycle):
        if not self.ser or not self.ser.is_open:
            print(f"No hay conexión activa en {self.serial_port} ({self.motor_id})")
            return
        try:
            # Duty cycle debe estar entre -1.0 y 1.0, se convierte a entero * 100000
            duty_int = int(duty_cycle * 100000)
            payload = bytearray([5])  # COMM_SET_DUTY
            payload.extend(struct.pack(">i", duty_int))
            packet = self.build_packet(payload)
            self.ser.write(packet)
        except serial.SerialException as e:
            print(f"Error al enviar duty cycle a {self.serial_port} ({self.motor_id}): {e}")

    @staticmethod
    def find_packet(response):
        if not response or len(response) < 5:
            print(f"Respuesta demasiado corta o vacía: {len(response)} bytes - {response.hex()}")
            return None
        
        for i in range(len(response) - 2):
            if response[i] == 2:
                length = response[i + 1]
                packet_end = i + 2 + length + 3
                if packet_end <= len(response) and response[packet_end - 1] == 3:
                    data = response[i + 2:i + 2 + length]
                    crc_received = (response[i + 2 + length] << 8) + response[i + 2 + length + 1]
                    crc_calculated = VESCController.crc16(data)
                    if crc_received == crc_calculated:
                        return data
        print("No se encontró un paquete válido en la respuesta.")
        return None

    def get_values(self, data_lock, vesc_data_list):
        if not self.ser or not self.ser.is_open:
            print(f"No hay conexión activa en {self.serial_port} ({self.motor_id})")
            return None

        try:
            payload = bytes([4])  # COMM_GET_VALUES
            packet = self.build_packet(payload)
            self.ser.write(packet)

            self.ser.reset_input_buffer()
            time.sleep(0.1)  # Aumentado para mayor estabilidad
            response = self.ser.read(100)

            if not response:
                print(f"No se recibió respuesta de {self.serial_port} ({self.motor_id}).")
                return None

            data = self.find_packet(response)
            if not data:
                print(f"No se extrajeron datos válidos de {self.serial_port} ({self.motor_id}).")
                return None

           #print(f"Longitud de datos: {len(data)} bytes")
           #print(f"Datos recibidos: {data.hex()}")

            if len(data) < 54:
                print(f"Datos demasiado cortos: {len(data)} bytes, esperado al menos 54")
                return None
            if len(data) > 58:
                print(f"Byte 58: {data[58]:02x} (decimal: {data[58]})")
            else:
                print(f"El byte 58 no existe, datos tienen solo {len(data)} bytes")
            print("-------------------------------------------------------")

            vesc_data = {
                "input_voltage": struct.unpack(">h", data[27:29])[0] / 10.0,
                "current_in": struct.unpack(">i", data[9:13])[0] / 100.0,
                "duty_cycle": struct.unpack(">h", data[21:23])[0] / 1000.0,
                "rpm": struct.unpack(">i", data[23:27])[0],
                "fault_code": struct.unpack(">B", data[53:54])[0],
            }
            
            # Mostrar solo fault_code, rpm, duty_cycle
            print(f"{self.motor_id}:")
            print(f"  fault_code: {vesc_data['fault_code']}")
            print(f"  rpm: {vesc_data['rpm']}")
           #print(f"  duty_cycle: {vesc_data['duty_cycle']:.3f}")

            # Actualizar la lista compartida
            with data_lock:
                vesc_data_list[self.motor_id] = vesc_data

            return vesc_data
        except (serial.SerialException, struct.error) as e:
            print(f"Error al obtener valores de {self.serial_port} ({self.motor_id}): {e}")
            return None

    def vesc_loop(self, target_rpm, data_lock, vesc_data_list, ramp_time=5.0, k=2.0, sign=1, control_mode="rpm", target_duty_cycle=None):
        if not self.ser or not self.ser.is_open:
            print(f"No hay conexión activa en {self.serial_port} ({self.motor_id})")
            return
        try:
            self.running = True
            start_time = time.time()
            while self.running:
                elapsed_time = time.time() - start_time
                
                if control_mode == "duty_cycle" and target_duty_cycle is not None:
                    # Control por duty cycle con rampa
                    current_duty = target_duty_cycle * (1 - math.exp(-k * elapsed_time / ramp_time))
                    if abs(current_duty) >= abs(target_duty_cycle):
                        current_duty = target_duty_cycle
                    
                    self.send_duty_cycle(current_duty * sign)
                else:
                    # Control por RPM (comportamiento original)
                    current_rpm = int(target_rpm * (1 - math.exp(-k * elapsed_time / ramp_time)))
                    if current_rpm >= target_rpm:
                        current_rpm = target_rpm
                    
                    self.send_rpm(current_rpm * sign)
                
                self.get_values(data_lock, vesc_data_list)
                
                # Ajustar delay según el modo de control
                if control_mode == "duty_cycle":
                    time.sleep(0.1 if abs(current_duty) >= abs(target_duty_cycle) else 0.01)
                else:
                    time.sleep(0.1 if current_rpm >= target_rpm else 0.01)
        except serial.SerialException as e:
            print(f"Error en el bucle de {self.serial_port} ({self.motor_id}): {e}")
        except KeyboardInterrupt:
            print(f"Bucle detenido para {self.serial_port} ({self.motor_id}).")
        except Exception as e:
            print(f"Error inesperado en el bucle de {self.serial_port} ({self.motor_id}): {e}")
        finally:
            self.running = False

    # Funciones estáticas corregidas con offsets adecuados
    @staticmethod
    def temp_mos1(data):
        return struct.unpack(">h", data[59:61])[0] / 10.0 if len(data) >= 61 else 0.0

    @staticmethod
    def temp_mos2(data):
        return struct.unpack(">h", data[61:63])[0] / 10.0 if len(data) >= 63 else 0.0

    @staticmethod
    def temp_mos3(data):
        return struct.unpack(">h", data[63:65])[0] / 10.0 if len(data) >= 65 else 0.0

    @staticmethod
    def current_motor(data):
        return struct.unpack(">i", data[5:9])[0] / 100.0 if len(data) >= 9 else 0.0

    @staticmethod
    def current_in(data):
        return struct.unpack(">i", data[9:13])[0] / 100.0 if len(data) >= 13 else 0.0

    @staticmethod
    def duty_cycle(data):
        return struct.unpack(">h", data[21:23])[0] / 1000.0 if len(data) >= 23 else 0.0

    @staticmethod
    def rpm(data):
        return struct.unpack(">i", data[23:27])[0] if len(data) >= 27 else 0

    @staticmethod
    def amp_hours(data):
        return struct.unpack(">i", data[29:33])[0] / 10000.0 if len(data) >= 33 else 0.0

    @staticmethod
    def amp_hours_charged(data):
        return struct.unpack(">i", data[33:37])[0] / 10000.0 if len(data) >= 37 else 0.0

    @staticmethod
    def tachometer(data):
        return struct.unpack(">i", data[45:49])[0] if len(data) >= 49 else 0

    @staticmethod
    def tachometer_abs(data):
        return struct.unpack(">i", data[49:53])[0] if len(data) >= 53 else 0

    @staticmethod
    def fault_code(data):
        return struct.unpack(">B", data[53:54])[0] if len(data) >= 54 else 0

def signal_handler(sig, frame, vescs, data_lock, vesc_data_list):
    print("\nInterrupción detectada, deteniendo comunicación con VESC...")
    with data_lock:
        for vesc, _, _ in vescs:
            vesc.running = False
    sys.exit(0)

def aggregate_data(data_lock, vesc_data_list, running_flag):
    while running_flag[0]:
        with data_lock:
            if not vesc_data_list:
                print("No hay datos disponibles para promediar/sumar.")
            else:
                voltages = [data["input_voltage"] for data in vesc_data_list.values()]
                currents = [data["current_in"] for data in vesc_data_list.values()]
                avg_voltage = sum(voltages) / len(voltages) if voltages else 0.0
                total_current = sum(currents) if currents else 0.0
                print(f"Voltaje promedio: {avg_voltage:.2f} V")
                print(f"Amperaje total: {total_current:.2f} A")
        time.sleep(1.0)

if __name__ == "__main__":
    data_lock = threading.Lock()
    vesc_data_list = defaultdict(dict)
    running_flag = [True]
    aggregator_thread = None  # Inicializar para evitar NameError

    vescs = [
        (VESCController(port, f"Motor {i+1}", BAUDRATE), config["direction"], config.get("target_rpm", 0), config.get("control_mode", "rpm"), config.get("target_duty_cycle", None))
        for i, (port, config) in enumerate(SERIAL_PORTS.items())
    ]

    try:
        # Registrar el manejador de señales
        signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, vescs, data_lock, vesc_data_list))

        # Conectar a todos los VESCs
        for vesc, _, _, _, _ in vescs:
            if not vesc.connect():
                print(f"No se pudo conectar a {vesc.serial_port} ({vesc.motor_id}), terminando...")
                sys.exit(1)

        # Iniciar hilo de agregación
        aggregator_thread = threading.Thread(
            target=aggregate_data,
            args=(data_lock, vesc_data_list, running_flag)
        )
        aggregator_thread.start()

        print("Iniciando comunicación con VESC...")
        threads = [
            threading.Thread(
                target=vesc.vesc_loop,
                args=(target_rpm, data_lock, vesc_data_list, 5.0, 2.0, direction, control_mode, target_duty_cycle)
            )
            for vesc, direction, target_rpm, control_mode, target_duty_cycle in vescs
        ]

        for thread in threads:
            thread.start()

        print("Comunicación con VESC iniciada. Presiona Ctrl+C para detener.")
        for thread in threads:
            thread.join()

    except Exception as e:
        print(f"Ocurrió un error: {e}")
    finally:
        print("Deteniendo comunicación con VESC...")
        running_flag[0] = False
        for vesc, _, _, _, _ in vescs:
            vesc.running = False
            vesc.disconnect()
        if aggregator_thread:
            aggregator_thread.join()
        print("Comunicación con VESC detena.")
