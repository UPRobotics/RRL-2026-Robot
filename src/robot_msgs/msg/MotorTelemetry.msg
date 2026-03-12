uint8   motor_id        # VESC controller ID reported by firmware
string  motor_name      # Human-readable motor name
int32   rpm             # Current RPM measured by VESC
float32 duty_cycle      # Current duty cycle (0.0 to 1.0)
float32 voltage         # Input bus voltage (Volts)
uint8   control_mode    # 0 = RPM mode, 1 = duty cycle mode
bool    inverted        # Motor orientation (true = electrically inverted)
