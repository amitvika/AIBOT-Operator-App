# Neck Actuator Controller

ESP32-based CAN bus controller for the neck actuator system. This provides **open-loop** motor control for up/down motion of the robot's neck.

## 🎯 Overview

The neck actuator is a simplified, single-motor controller that responds to U (up) and D (down) commands via CAN bus. Unlike the wheel actuators, this operates in **open-loop mode** with no position feedback - it simply controls motor speed proportionally to the input commands.

## 📡 CAN Bus Configuration

- **Default CAN ID**: `0x500` (configurable via tuning)
- **CAN Bitrate**: 500 kbps
- **Message Format**: 8-byte payload `[f, b, l, r, u, d, l2, r2]`
- **Relevant Commands**: 
  - Byte 4: `u` (Up, 0-100)
  - Byte 5: `d` (Down, 0-100)

## 🔧 Hardware Connections

### CAN Bus Pins
- **TX Pin**: GPIO 48
- **RX Pin**: GPIO 34
- **STBY Pin**: GPIO 47

### Motor Pins
- **Forward (Up)**: GPIO 37
- **Reverse (Down)**: GPIO 38

### PWM Configuration
- **Frequency**: 20 kHz
- **Resolution**: 8-bit (0-255)

## 🎮 Operation

### Motor Control
The neck actuator responds to U and D commands:
- **U value (0-100)**: Controls upward motion speed
- **D value (0-100)**: Controls downward motion speed
- **Speed calculation**: `motor_speed = (U - D) / 100`
  - Positive speed → Motor moves up
  - Negative speed → Motor moves down
  - Zero → Motor stops

### Example Commands
```
U=100, D=0   → Full speed up
U=50, D=0    → Half speed up
U=0, D=100   → Full speed down
U=0, D=50    → Half speed down
U=0, D=0     → Stop
```

## 🛠️ Tuning Commands

The neck actuator supports the following tuning commands via CAN:

| Command | CAN ID | Parameters | Description |
|---------|--------|------------|-------------|
| Set Address | 0x300 | `address(2)` | Change CAN address |
| Motor Direction | 0x302 | `address(2), inverted(1)` | Invert motor direction |
| Max PWM | 0x305 | `address(2), max_pwm(1)` | Set maximum PWM (0-255) |
| Min PWM | 0x306 | `address(2), min_pwm(1)` | Set minimum PWM (deadzone) |
| Save Parameters | 0x307 | `address(2)` | Save to EEPROM |
| Request Telemetry | 0x308 | `address(2)` | Request current values |

### Addressing
- Use `0xFFFF` as broadcast address to target all actuators
- Use specific address (e.g., `0x500`) to target individual actuator

## 📊 Telemetry

The neck actuator can report its current configuration via CAN:

**Telemetry Response (CAN ID: 0x309)**
```
Byte 0-1: CAN Address (uint16, big-endian)
Byte 2:   Motor Direction Inverted (0=normal, 1=inverted)
Byte 3:   Reserved (0)
Byte 4:   Max PWM value (0-255)
Byte 5:   Min PWM value (0-255)
Byte 6-7: Reserved (0)
```

## 💾 Persistent Storage

All tuning parameters are saved to EEPROM and automatically loaded on startup:
- CAN Address
- Motor Direction Inversion
- Max PWM
- Min PWM

## 🔍 Debugging

### Serial Monitor Output
The controller outputs detailed debug information at 115200 baud:
- Motor speed updates
- CAN message reception
- Tuning command processing
- Telemetry transmission

### Example Output
```
--- Neck Actuator CAN Controller (Listening for ID: 0x500) ---
=== Loaded Parameters from EEPROM ===
  CAN Address: 0x500
  Motor Inverted: NO
  Max PWM: 255
  Min PWM: 30
======================================
CAN Bus initialized successfully.
Motor PWM outputs configured.

-> Drive Cmd: U=75, D=0 -> Speed=0.75
Motor Speed: 0.75 (75%)

-> Tuning Cmd ID: 0x308, DLC: 2
-> Telemetry requested for address 0x500
-> Telemetry sent
```

## 🚨 CAN Bus Health Monitoring

The controller includes automatic CAN bus health monitoring:
- **Bus-off detection**: Automatically initiates recovery
- **Error counter monitoring**: Logs warnings when TX/RX errors exceed threshold
- **Auto-restart**: Restarts stopped CAN bus automatically

## 🔄 Comparison with Wheel Actuator

| Feature | Wheel Actuator | Neck Actuator |
|---------|----------------|---------------|
| Control Mode | Closed-loop (PID) | Open-loop |
| Sensor | AS5047P encoder | None |
| Motors | 2 (Drive + Position) | 1 (Up/Down) |
| Commands | F, B, L, R | U, D |
| Position Control | Yes | No |
| Telemetry Messages | 2 (full data) | 1 (simplified) |

## 📝 Notes

- **Open-loop operation**: No position feedback means the actuator will continue moving as long as U or D is non-zero
- **Safety**: Ensure mechanical limits are in place to prevent over-travel
- **PWM deadzone**: The `min_pwm` parameter accounts for DC motor deadzone (default: 30)
- **Direction inversion**: Use motor direction inversion if the actuator moves opposite to expected

## 🔧 Tuning Tips

1. **Max PWM**: Reduce if motor is too fast or draws too much current
2. **Min PWM**: Adjust to ensure motor starts moving smoothly
3. **Motor Direction**: Invert if up/down is reversed from expected
4. **CAN Address**: Change if running multiple neck actuators on the same bus

