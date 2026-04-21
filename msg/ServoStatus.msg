# HerkuleX Servo Status Message
uint8 servo_id          # Servo ID (0~253)
int32 position          # Current position (0~1023)
float32 angle           # Current angle (-160.0 ~ 160.0 degrees)
int32 speed             # Current speed (-1023 ~ 1023)
uint8 status_error      # Status error byte
uint8 status_detail     # Status detail byte
