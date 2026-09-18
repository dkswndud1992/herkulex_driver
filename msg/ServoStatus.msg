# HerkuleX Servo Status Message
uint8 servo_id          # Servo ID (0~253)
int32 position          # Current position (0~1023)
float32 angle           # Current angle (-160.0 ~ 160.0 degrees)
int32 speed             # Current speed (-1023 ~ 1023)
uint8 status_error      # Status error byte
uint8 status_detail     # Status detail byte

# Extended real-time telemetry (streamed from RAM block read)
uint8 voltage           # Supply voltage (raw value, actual = raw * 0.074V)
uint8 temperature       # Servo temperature (raw ADC value)
int32 pwm               # Current PWM output value
uint8 torque_control    # Torque state (0x00=Free, 0x40=Brake, 0x60=On)
