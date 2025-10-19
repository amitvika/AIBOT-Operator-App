# Joint Actuator Controller

ESP32-based CAN bus controller for robot arm joint actuators. This provides **closed-loop position control** using PID for precise angular positioning.

## 🎯 Overview

The joint actuator is a single-motor controller with position feedback from an AS5047P magnetic encoder. It uses PID control to precisely reach and maintain target positions sent via CAN bus. This is ideal for robot arm joints that require accurate positioning.

## 📡 CAN Bus Configuration

- **CAN ID Range**: `0x600-0x60B` (12 actuators supported)
  - Joint 1: 0x600 (1536)
  - Joint 2: 0x601 (1537)
  - Joint 3: 0x602 (1538)
  - ...
  - Joint 12: 0x60B (1547)
- **CAN Bitrate**: 500 kbps
- **Position Command**: 4-byte float (degrees)
- **Default CAN ID**: `0x600` (must be configured for each actuator)

## 🔧 Hardware Connections

### CAN Bus Pins
- **TX Pin**: GPIO 48
- **RX Pin**: GPIO 34
- **STBY Pin**: GPIO 47

### Motor Pins
- **Forward**: GPIO 37
- **Reverse**: GPIO 38

### AS5047P Encoder (SPI)
- **CS**: GPIO 4
- **SCK**: GPIO 12
- **MISO**: GPIO 13
- **MOSI**: GPIO 14

### PWM Configuration
- **Frequency**: 20 kHz
- **Resolution**: 8-bit (0-255)

## 🎮 Operation

### Position Control
The joint actuator uses a PID controller to reach target positions:
- **Input**: Target position in degrees (0-360°)
- **Sensor**: AS5047P 14-bit magnetic encoder (0.022° resolution)
- **Output**: PWM-controlled motor speed
- **Control Loop**: 50ms update rate

### How It Works
```
Target Position → PID Controller → Motor Output → AS5047P Sensor → Feedback
                      ↑_______________|
```

### Position Command Format
```
CAN Message:
  ID: Joint actuator address (0x600-0x60B)
  DLC: 4 bytes minimum
  Data: [target_position (float, 4 bytes)]
  
Example: Set joint 1 to 90 degrees
  ID: 0x600
  Data: 0x00 0x00 0xB4 0x42  (90.0 as little-endian float)
```

## 🛠️ Tuning Commands

All standard tuning commands are supported via CAN:

| Command | CAN ID | Parameters | Description |
|---------|--------|------------|-------------|
| Set Address | 0x300 | `address(2)` | Change CAN address |
| Set Setpoint | 0x301 | `address(2), setpoint(4)` | Set zero position offset |
| Motor Direction | 0x302 | `address(2), inverted(1)` | Invert motor direction |
| Sensor Direction | 0x303 | `address(2), inverted(1)` | Invert sensor direction |
| Update P Gain | 0x304 | `address(2), p_gain(4)` | Set proportional gain |
| Max PWM | 0x305 | `address(2), max_pwm(1)` | Set maximum PWM (0-255) |
| Min PWM | 0x306 | `address(2), min_pwm(1)` | Set minimum PWM (deadzone) |
| Save Parameters | 0x307 | `address(2)` | Save to EEPROM |
| Request Telemetry | 0x308 | `address(2)` | Request current values |

### Addressing
- Use `0xFFFF` as broadcast address to target all actuators
- Use specific address (e.g., `0x600`) to target individual actuator

## 📊 Telemetry

The joint actuator reports its configuration via two CAN messages:

**Telemetry Response 1 (CAN ID: 0x309)**
```
Byte 0-1: CAN Address (uint16, big-endian)
Byte 2:   Motor Direction Inverted (0=normal, 1=inverted)
Byte 3:   Sensor Direction Inverted (0=normal, 1=inverted)
Byte 4:   Max PWM value (0-255)
Byte 5:   Min PWM value (0-255)
```

**Telemetry Response 2 (CAN ID: 0x30A)**
```
Byte 0-3: Setpoint (float, little-endian, degrees)
Byte 4-7: P Gain (float, little-endian)
```

## 💾 Persistent Storage

All tuning parameters are saved to EEPROM and automatically loaded on startup:
- CAN Address
- Setpoint (zero position offset)
- Motor Direction Inversion
- Sensor Direction Inversion
- P Gain
- Max PWM
- Min PWM

## 🎯 PID Tuning Guide

### Understanding PID Parameters

**P (Proportional) Gain:**
- Controls how aggressively the actuator moves toward target
- **Too Low**: Slow response, position error
- **Too High**: Oscillation, instability
- **Typical Range**: 3.0 - 10.0
- **Default**: 5.0

**I (Integral) Gain:**
- Currently set to 0 (not used)
- Can be enabled in code if needed for steady-state error correction

**D (Derivative) Gain:**
- Currently set to 0 (not used)
- Can be enabled in code for damping oscillations

### Tuning Procedure

1. **Start with Default P=5.0**
   - Test basic movement
   - Observe response

2. **Adjust P Gain**
   - If too slow: Increase P (try 7.0)
   - If oscillating: Decrease P (try 3.0)
   - Fine-tune in small steps (0.5)

3. **Adjust PWM Limits**
   - **Max PWM**: Limit maximum speed
   - **Min PWM**: Overcome motor friction/deadzone
   - Typical: Max=255, Min=30

4. **Set Setpoint (Zero Position)**
   - Read current position at desired "zero"
   - Set setpoint to that value
   - All commands now relative to this zero

### Example Tuning Session
```
1. Upload firmware with default P=5.0
2. Send target position = 90°
3. Observe: Joint overshoots and oscillates
4. Reduce P to 3.5
5. Test again: Slower but stable
6. Increase slightly to 4.0
7. Perfect! Save parameters
```

## 🔍 Debugging

### Serial Monitor Output
The controller outputs detailed debug information at 115200 baud:
- Position readings
- Target position
- Position error
- Motor output (PID result)
- CAN message reception
- Tuning command processing

### Example Output
```
--- Joint Actuator Controller (CAN ID: 0x600) ---
=== Loaded Parameters from EEPROM ===
  CAN Address: 0x600
  Setpoint: 0.00 degrees
  Motor Inverted: NO
  Sensor Inverted: NO
  P Gain: 5.000
  Max PWM: 255
  Min PWM: 30
======================================
CAN Bus initialized successfully.
AS5047P Position Sensor initialized.

-> Position Cmd: Target=90.00°
Pos: 5.23° | Target: 90.00° | Error: 84.77° | Output: 1.00
Pos: 22.15° | Target: 90.00° | Error: 67.85° | Output: 1.00
Pos: 45.88° | Target: 90.00° | Error: 44.12° | Output: 1.00
Pos: 72.34° | Target: 90.00° | Error: 17.66° | Output: 0.88
Pos: 88.92° | Target: 90.00° | Error: 1.08° | Output: 0.05
Pos: 89.95° | Target: 90.00° | Error: 0.05° | Output: 0.00
```

## 🚨 CAN Bus Health Monitoring

The controller includes automatic CAN bus health monitoring:
- **Bus-off detection**: Automatically initiates recovery
- **Error counter monitoring**: Logs warnings when TX/RX errors exceed threshold
- **Auto-restart**: Restarts stopped CAN bus automatically
- **1-second health check interval**

## 🔄 Comparison with Other Actuators

| Feature | Wheel Actuator | Joint Actuator | Neck Actuator |
|---------|----------------|----------------|---------------|
| Control Mode | Closed-loop (2 motors) | Closed-loop (1 motor) | Open-loop |
| Sensor | AS5047P | AS5047P | None |
| Motors | 2 (Drive + Position) | 1 (Position) | 1 (Speed) |
| PID Control | Yes (M2 only) | Yes | No |
| Commands | F, B, L, R | Position (float) | U, D |
| Telemetry | 2 messages | 2 messages | 1 message |
| Use Case | Mobile base | Arm joints | Neck tilt |

## 📝 Important Notes

### Position Wrapping
- The AS5047P sensor provides 360° absolute position
- Values wrap at 360° (0° = 360°)
- For multi-turn applications, add external counting

### Setpoint (Zero Position)
- The setpoint defines where "zero degrees" is
- All target positions are relative to this zero
- Set it at your desired home position
- Example: If sensor reads 45° at home, set setpoint=45°

### Motor/Sensor Direction
- Use inversion if joint moves opposite to expected
- **Motor Inversion**: Reverses motor direction
- **Sensor Inversion**: Reverses encoder counting
- Usually only one needs inversion, not both

### Power Considerations
- Ensure adequate power supply for motor
- Use separate power for motor driver (not ESP32 5V)
- Typical current: 1-3A peak depending on motor

## 🔧 Configuration for Multiple Joints

### CAN Address Assignment
```cpp
// Before uploading to each ESP32, change:
const int JOINT_ACTUATOR_CAN_ID = 0x600;  // Joint 1
const int JOINT_ACTUATOR_CAN_ID = 0x601;  // Joint 2
const int JOINT_ACTUATOR_CAN_ID = 0x602;  // Joint 3
// ... etc for all 12 joints
```

### Or Use Tuning Command
```
1. Flash all actuators with default 0x600
2. Power on one at a time
3. Use CAN address tuning to set unique ID
4. Save to EEPROM
5. Power cycle to verify
```

## 🎮 Integration with Control System

### Sending Position Commands
```python
# Example Python code for RPI4
import struct
import can

bus = can.interface.Bus(channel='can0', interface='socketcan')

# Send position command to joint 1
joint_id = 0x600
target_position = 90.0  # degrees

# Pack float as little-endian
data = struct.pack('<f', target_position)

msg = can.Message(
    arbitration_id=joint_id,
    data=data,
    is_extended_id=False
)

bus.send(msg)
```

### Reading Joint Positions
Currently, the joint actuators send position via telemetry on request. For continuous position feedback, consider adding a periodic position broadcast feature.

## 💡 Advanced Features (Future Enhancements)

### Velocity Control
Add velocity limiting in PID:
```cpp
// Limit rate of change of motor output
float max_velocity = 0.1;  // per loop
if (abs(new_output - old_output) > max_velocity) {
  new_output = old_output + sign(new_output - old_output) * max_velocity;
}
```

### Position Limits
Add software limits:
```cpp
const float MIN_POSITION = 0.0;
const float MAX_POSITION = 180.0;

if (g_target_position_degrees < MIN_POSITION) {
  g_target_position_degrees = MIN_POSITION;
}
if (g_target_position_degrees > MAX_POSITION) {
  g_target_position_degrees = MAX_POSITION;
}
```

### Current Sensing
Add current measurement for:
- Overload detection
- Collision detection
- Force control

## 📋 Testing Checklist

- [ ] ESP32 powers on and shows serial output
- [ ] CAN bus initializes successfully
- [ ] AS5047P sensor provides valid readings
- [ ] Motor responds to position commands
- [ ] Position error decreases to near zero
- [ ] PID controller is stable (no oscillation)
- [ ] Telemetry retrieval works
- [ ] Parameters save to EEPROM
- [ ] Parameters load correctly after power cycle
- [ ] All 12 joints have unique CAN addresses
- [ ] No CAN bus conflicts between joints

---

**Ready to build your robot arm!** This joint actuator provides precise, reliable position control for all your robotic joints. 🤖

