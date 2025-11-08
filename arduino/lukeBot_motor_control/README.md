# Arduino Motor Control Firmware

Arduino Mega firmware for controlling Mecanum wheels via L298N motor drivers.

## Hardware Setup

### Motor Connections
- **L298N #1** (Left side): Front-left (FL) and Rear-left (RL) motors
- **L298N #2** (Right side): Front-right (FR) and Rear-right (RR) motors

### Pin Connections
**L298N #1:**
- IN1 → Arduino Pin 2
- IN2 → Arduino Pin 3
- IN3 → Arduino Pin 4
- IN4 → Arduino Pin 5
- ENA → Arduino Pin 6 (PWM)
- ENB → Arduino Pin 7 (PWM)
- GND → Arduino GND

**L298N #2:**
- IN1 → Arduino Pin 8
- IN2 → Arduino Pin 9
- IN3 → Arduino Pin 10
- IN4 → Arduino Pin 11
- ENA → Arduino Pin 12 (PWM)
- ENB → Arduino Pin 13 (PWM)
- GND → Arduino GND

## Upload Instructions

1. Open Arduino IDE
2. Open `lukeBot_motor_control.ino`
3. Select board: **Arduino Mega 2560**
4. Select correct COM port (Tools → Port)
5. Click Upload

## Serial Communication

- **Baud Rate**: 115200
- **Protocol**: ASCII commands with newline terminator

## Commands

The Arduino accepts the following high-level commands:

- `MOVE_FORWARD,<speed>` - Move forward (speed: 0-100)
- `MOVE_BACKWARD,<speed>` - Move backward (speed: 0-100)
- `TURN_LEFT,<angle>` - Turn left (angle in degrees)
- `TURN_RIGHT,<angle>` - Turn right (angle in degrees)
- `STOP` - Stop all motors

## Responses

The Arduino sends responses:
- `READY` - On startup
- `OK: <command>` - Command executed successfully
- `ERROR: <message>` - Command error

## Testing

After uploading, open Serial Monitor (115200 baud) and send commands:
- `MOVE_FORWARD,50`
- `STOP`
- `TURN_LEFT,90`
- `STOP`

## Notes

- Motor speed is automatically scaled from 0-100 to PWM range (MIN_SPEED-255)
- Minimum speed (30) is used to overcome friction
- All motors stop initially on startup
- Angle-based turning is currently a placeholder - TODO: implement with odometry

