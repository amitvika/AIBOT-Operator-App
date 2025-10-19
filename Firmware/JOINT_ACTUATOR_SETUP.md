# Joint Actuator - Quick Setup Guide

## 🎯 Overview

This guide will help you set up all 12 joint actuators for your robot arm. Each actuator provides precise position control using PID and an AS5047P encoder.

## 📋 What You Need

- **12x ESP32 boards**
- **12x H-bridge motor drivers**
- **12x DC motors with encoders**
- **12x AS5047P magnetic encoder modules**
- **CAN bus network** (with 120Ω termination)
- **Power supplies** (5V for ESP32, motor voltage for drivers)

## 🔢 CAN Address Assignment

Each joint actuator needs a unique CAN address:

| Joint # | CAN ID | Decimal | Hex | Actuator |
|---------|--------|---------|-----|----------|
| Joint 1 | 0x600 | 1536 | 600 | Shoulder Pan |
| Joint 2 | 0x601 | 1537 | 601 | Shoulder Tilt |
| Joint 3 | 0x602 | 1538 | 602 | Elbow |
| Joint 4 | 0x603 | 1539 | 603 | Wrist Roll |
| Joint 5 | 0x604 | 1540 | 604 | Wrist Pitch |
| Joint 6 | 0x605 | 1541 | 605 | Wrist Yaw |
| Joint 7 | 0x606 | 1542 | 606 | Gripper Rotate |
| Joint 8 | 0x607 | 1543 | 607 | Reserved |
| Joint 9 | 0x608 | 1544 | 608 | Reserved |
| Joint 10 | 0x609 | 1545 | 609 | Reserved |
| Joint 11 | 0x60A | 1546 | 60A | Reserved |
| Joint 12 | 0x60B | 1547 | 60B | Reserved |

## 🚀 Method 1: Pre-Configure Each Actuator

### Step 1: Edit Code for Each Joint
```cpp
// In jointActuator.cpp, change for each actuator:
const int JOINT_ACTUATOR_CAN_ID = 0x600;  // Joint 1
const int JOINT_ACTUATOR_CAN_ID = 0x601;  // Joint 2
const int JOINT_ACTUATOR_CAN_ID = 0x602;  // Joint 3
// ... etc
```

### Step 2: Upload to Each ESP32
```bash
# Label each ESP32 (Joint 1, Joint 2, etc.)
# Upload corresponding firmware to each one
pio run --target upload
```

### Step 3: Verify Address
```bash
# Open serial monitor
pio device monitor -b 115200

# Look for:
--- Joint Actuator Controller (CAN ID: 0x600) ---
  CAN Address: 0x600  # Should match your joint number
```

## 🚀 Method 2: Flash All, Then Configure via CAN

### Step 1: Flash All with Default (0x600)
```bash
# Upload same firmware to all 12 ESP32s
pio run --target upload
```

### Step 2: Configure One at a Time
```
1. Power on ONLY Joint 1 → Already 0x600 ✓
2. Power on Joint 2
   - Use web UI to set address to 0x601
   - Save to EEPROM
   - Power cycle to verify
3. Power on Joint 3
   - Set to 0x602, save, verify
4. Repeat for all 12 joints
```

## 🔌 Hardware Setup (Per Actuator)

### ESP32 Connections
```
CAN Bus:
  TX Pin   → GPIO 48
  RX Pin   → GPIO 34
  STBY Pin → GPIO 47
  
Motor Driver:
  Forward  → GPIO 37
  Reverse  → GPIO 38
  
AS5047P Encoder (SPI):
  CS   → GPIO 4
  SCK  → GPIO 12
  MISO → GPIO 13
  MOSI → GPIO 14
```

### Power
- **ESP32**: 5V (USB or external regulator)
- **Motor Driver**: Appropriate motor voltage (12V, 24V, etc.)
- **Ground**: All grounds connected together

### CAN Bus
- **Daisy chain** all actuators on CAN bus
- **120Ω termination** at both ends of bus
- **Twisted pair** cable for CAN-H and CAN-L

## ⚙️ Initial Calibration (Per Joint)

### 1. Test Motor Direction
```
1. Send position command: +90°
2. Watch which way motor turns
3. If wrong direction:
   - Use web UI to invert motor direction
   - Or swap GPIO 37 ↔ GPIO 38
```

### 2. Test Sensor Direction
```
1. Manually rotate motor shaft clockwise
2. Check serial monitor: position should increase
3. If decreasing:
   - Use web UI to invert sensor direction
```

### 3. Set Zero Position
```
1. Move joint to desired home/zero position
2. Read current position from serial monitor
3. Set that value as setpoint via web UI
4. Example: If sensor reads 45° at home, setpoint=45°
```

### 4. Tune PID
```
1. Start with default P=5.0
2. Send position command (e.g., 90°)
3. Observe response:
   - Too slow? Increase P
   - Oscillating? Decrease P
4. Fine-tune in steps of 0.5
```

### 5. Adjust PWM Limits
```
Max PWM (default 255):
  - Reduce if motor is too fast
  - Typical: 200-255
  
Min PWM (default 30):
  - Increase if motor doesn't move smoothly
  - Find minimum value that overcomes friction
  - Typical: 20-50
```

### 6. Save Configuration
```
1. Use web UI "Save All" button
2. Power cycle
3. Verify settings loaded from EEPROM
```

## 🧪 Testing Procedure

### Individual Joint Test
```bash
# 1. Open serial monitor
pio device monitor -b 115200

# 2. Send position command via RPI4/control system
# Joint should move to target position

# 3. Monitor output:
Pos: 5.23° | Target: 90.00° | Error: 84.77° | Output: 1.00
Pos: 45.88° | Target: 90.00° | Error: 44.12° | Output: 1.00
Pos: 89.95° | Target: 90.00° | Error: 0.05° | Output: 0.00
```

### Full Arm Test
```
1. Power on all 12 joints
2. Verify no CAN address conflicts
3. Send position commands to each joint sequentially
4. Verify each joint responds only to its address
5. Test simultaneous movement of multiple joints
```

## 📊 Default Parameters

```cpp
P Gain:         5.0
I Gain:         0.0
D Gain:         0.0
Max PWM:        255
Min PWM:        30
Setpoint:       0.0°
Motor Inv:      false
Sensor Inv:     false
```

## 🐛 Troubleshooting

### Joint doesn't respond to commands
- ✓ Check CAN address matches command
- ✓ Verify CAN bus wiring (TX/RX not swapped)
- ✓ Check termination resistors (120Ω)
- ✓ Look for CAN errors in serial monitor

### Joint moves but doesn't reach target
- ✓ Increase P gain
- ✓ Check motor has enough power
- ✓ Verify no mechanical binding
- ✓ Check encoder is working (position changes)

### Joint oscillates around target
- ✓ Decrease P gain
- ✓ Check for mechanical play/backlash
- ✓ Reduce max PWM

### Position readings are wrong
- ✓ Check AS5047P wiring (SPI)
- ✓ Verify magnet is centered over sensor
- ✓ Check magnet-to-sensor distance (1-3mm)
- ✓ Try inverting sensor direction

### CAN address conflicts
- ✓ Each joint must have unique ID
- ✓ Use serial monitor to verify loaded address
- ✓ Check EEPROM values
- ✓ Re-flash and reconfigure if needed

## 📝 Configuration Worksheet

Print and fill out for each joint:

```
Joint #: ___  CAN ID: 0x___  Decimal: ____

Hardware:
  [ ] ESP32 labeled
  [ ] Wired correctly
  [ ] Powers on
  [ ] CAN bus connected

Calibration:
  [ ] Motor direction correct
  [ ] Sensor direction correct
  [ ] Zero position set: ____°
  [ ] P gain tuned: ____
  [ ] Max PWM: ____
  [ ] Min PWM: ____
  [ ] Saved to EEPROM

Testing:
  [ ] Responds to position commands
  [ ] Reaches target accurately
  [ ] No oscillation
  [ ] Telemetry works

Notes:
_________________________________
_________________________________
```

## 🎯 Quick Commands Reference

### Via Web UI (webrtc.html)
1. Select joint from dropdown (Joint 1-12)
2. Click "📥 Retrieve" to load values
3. Adjust parameters
4. Click "📤 Send" for each parameter
5. Click "💾 Save All" to persist

### Via Serial Monitor
- Watch position updates every 50ms
- See PID calculations
- Monitor CAN messages
- View tuning changes

### Via RPI4 (Python)
```python
import can
import struct

bus = can.interface.Bus('can0', interface='socketcan')

# Send position to joint 1
data = struct.pack('<f', 90.0)  # 90 degrees
msg = can.Message(arbitration_id=0x600, data=data)
bus.send(msg)
```

## ✅ Final Checklist

Before deploying your robot arm:

- [ ] All 12 joints have unique CAN addresses
- [ ] All joints respond to commands
- [ ] All joints reach target positions accurately
- [ ] No oscillation or instability
- [ ] All parameters saved to EEPROM
- [ ] Zero positions calibrated
- [ ] Mechanical limits checked
- [ ] No CAN bus errors
- [ ] Power supply adequate for all joints
- [ ] Emergency stop tested

## 🎉 You're Ready!

Once all 12 joints are configured and tested, your robot arm is ready for integration with your control system. The joints will work seamlessly with the existing wheel actuators and neck actuator on the same CAN bus.

---

**Next Steps:**
1. Integrate with motion planning system
2. Add trajectory generation
3. Implement inverse kinematics
4. Test full arm movements
5. Add safety features (collision detection, limits, etc.)

Good luck with your robot arm! 🤖

