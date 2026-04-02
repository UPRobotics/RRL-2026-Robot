uint8   motor_id        # VESC controller ID reported by firmware
string  motor_name      # Human-readable motor name
int32   rpm             # Current RPM measured by VESC
float32 duty_cycle      # Current duty cycle (-1.0 to 1.0)
float32 current_in      # Input current draw (Amps)
float32 voltage         # Input bus voltage (Volts)
float32 position        # Rotor position (degrees)
uint8   fault_code      # VESC fault code (0 = no fault)
uint8   control_mode    # 0 = RPM mode, 1 = duty cycle mode
bool    inverted        # Motor orientation (true = electrically inverted)
float32 current_motor      # Motor phase current (Amps)
float32 rpm_limit          # Configured RPM limit from config.json
float32 duty_cycle_limit   # Configured duty cycle limit from config.json
