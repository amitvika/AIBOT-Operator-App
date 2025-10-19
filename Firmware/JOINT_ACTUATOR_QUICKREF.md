# Joint Actuator - Quick Reference Card

## 📋 Quick Facts
- **File**: `jointActuator.cpp`
- **CAN IDs**: `0x600-0x60B` (1536-1547 decimal)
- **Quantity**: 12 joint actuators
- **Control**: Closed-loop PID position control
- **Sensor**: AS5047P 14-bit magnetic encoder
- **Motor**: Single DC motor with H-bridge

## 🔌 Pin Connections
```
CAN:     TX=48, RX=34, STBY=47
Motor:   FWD=37, REV=38
Encoder: CS=4, SCK=12, MISO=13, MOSI=14
```

## 🎮 CAN Address Map
```
Joint 1-6:  0x600-0x605 (Main arm)
Joint 7-12: 0x606-0x60B (Additional/reserved)
```

## 📡 Position Command Format
```
CAN Message:
  ID: Joint address (0x600-0x60B)
  Data: Target position (4-byte float, little-endian)
  
Example: 90 degrees = 0x00 0x00 0xB4 0x42
```

## ⚙️ Default Parameters
```
P Gain:    5.0
Max PWM:   255
Min PWM:   30
Setpoint:  0.0°
```

## 🔧 Calibration Steps
```
1. Flash firmware (set unique CAN ID)
2. Test motor direction
3. Test sensor direction  
4. Set zero position (setpoint)
5. Tune P gain (3.0-10.0)
6. Adjust PWM limits
7. Save to EEPROM
```

## 🎯 PID Tuning
```
Too Slow     → Increase P (try +1.0)
Oscillating  → Decrease P (try -1.0)
Perfect      → Error < 1° and stable
```

## 📊 Telemetry
```
Request:  CAN 0x308 with joint address
Response: CAN 0x309 + 0x30A (2-part)
  Part 1: Address, directions, PWM limits
  Part 2: Setpoint, P gain
```

## 🚀 Upload & Test
```bash
# Edit CAN ID for each joint first!
pio run --target upload

# Serial monitor
pio device monitor -b 115200

# Expected output
Pos: 89.95° | Target: 90.00° | Error: 0.05° | Output: 0.00
```

## 🐛 Quick Troubleshooting
| Problem | Quick Fix |
|---------|-----------|
| No response | Check CAN address |
| Wrong direction | Invert motor direction |
| Doesn't reach target | Increase P gain |
| Oscillates | Decrease P gain |
| Sensor not working | Check SPI wiring |

## 💡 Pro Tips
- **Label each ESP32** with joint number before assembly
- **Test individually** before connecting all to CAN bus
- **Use broadcast (0xFFFF)** to configure multiple at once
- **Save often** - power loss = lost settings
- **Start low P** - easier to increase than decrease

## 📚 Documentation
- **Full Reference**: `JOINT_ACTUATOR_README.md`
- **Setup Guide**: `JOINT_ACTUATOR_SETUP.md`
- **Calibration**: See setup guide section "Initial Calibration"

## 🎯 Position Control Loop
```
Target → [PID] → Motor → Encoder → Feedback
  ↑__________________________|
```

## 🔢 Address Assignment Table
| Joint | Hex | Dec | Purpose |
|-------|-----|-----|---------|
| 1 | 0x600 | 1536 | Shoulder Pan |
| 2 | 0x601 | 1537 | Shoulder Tilt |
| 3 | 0x602 | 1538 | Elbow |
| 4 | 0x603 | 1539 | Wrist Roll |
| 5 | 0x604 | 1540 | Wrist Pitch |
| 6 | 0x605 | 1541 | Wrist Yaw |
| 7-12 | 0x606-0x60B | 1542-1547 | Reserved |

## ⚡ Key Differences from Wheel Actuator
```
Wheel:  2 motors, drive + position
Joint:  1 motor, position only

Wheel:  Complex dual-motor control
Joint:  Simple single-motor PID

Wheel:  Mobile base application
Joint:  Arm positioning application
```

## 📝 Remember
- Each joint needs **unique CAN ID**
- Calibrate **zero position** for each joint
- **P gain** varies per joint (weight, inertia)
- **Save to EEPROM** after tuning
- Test **one at a time** then together

