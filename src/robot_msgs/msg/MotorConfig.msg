uint8   config_index       # Index of this motor in the motors[] array of config.json
uint8   motor_vesc_id      # VESC firmware CAN/UART controller ID (used by autoConnect)
string  motor_name         # Human-readable motor name (for display/validation)
float32 rpm_limit          # Maximum RPM (applied on both directions)
float32 duty_cycle_limit   # Maximum duty cycle, 0.0–1.0
uint8   control_mode       # 0 = RPM mode, 1 = duty cycle mode
bool    inverted           # Electrical orientation of the motor
