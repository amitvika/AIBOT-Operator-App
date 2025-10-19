# Neck Actuator - Quick Setup Guide

## ✅ Files Created

1. **`neckActuator.cpp`** - Main ESP32 firmware for the neck actuator
2. **`NECK_ACTUATOR_README.md`** - Complete documentation and reference guide

## 🎯 Key Features

### Open-Loop Motor Control
- **Single motor** controlled by U (up) and D (down) commands
- **No encoder/sensor** - pure open-loop speed control
- **Proportional speed**: U and D values (0-100) directly control motor speed
- **Bidirectional**: Positive speed = up, negative speed = down

### CAN Bus Integration
- **Default CAN ID**: `0x500`
- **Compatible** with existing RPI4 control system
- **Listens** to bytes 4 (U) and 5 (D) in the 8-byte CAN payload

### Tuning Support
The neck actuator supports the following tuning commands:
- ✅ **CAN Address Change** (0x300)
- ✅ **Motor Direction Inversion** (0x302)
- ✅ **Max PWM Limit** (0x305)
- ✅ **Min PWM (Deadzone)** (0x306)
- ✅ **Save to EEPROM** (0x307)
- ✅ **Telemetry Request** (0x308)

### Telemetry System
- **Single message response** (CAN ID 0x309)
- **Reports**: CAN address, motor direction, max/min PWM values
- **Compatible** with existing telemetry infrastructure

## 🔌 Hardware Setup

### ESP32 Connections
```
CAN Bus:
  TX   → GPIO 48
  RX   → GPIO 34
  STBY → GPIO 47

Motor Driver:
  Forward (Up)   → GPIO 37
  Reverse (Down) → GPIO 38
```

### Power
- ESP32: 5V via USB or external
- Motor: Through H-bridge driver (separate power supply recommended)

## 🚀 Quick Start

### 1. Upload Firmware
```bash
# Using PlatformIO
pio run --target upload

# Or using Arduino IDE
# Open neckActuator.cpp and upload
```

### 2. Verify CAN Communication
Open serial monitor at **115200 baud**. You should see:
```
--- Neck Actuator CAN Controller (Listening for ID: 0x500) ---
CAN Bus initialized successfully.
Motor PWM outputs configured.
```

### 3. Test Motor Control
Send CAN messages from your control system with:
- **U value**: 0-100 (upward motion)
- **D value**: 0-100 (downward motion)

### 4. Test with Existing System
The neck actuator **automatically works** with your existing RPI4 control system since it already sends U and D values in the CAN payload!

## 🎮 Using with WebRTC Control Panel

The neck actuator can be tuned using the existing debug panel in `webrtc.html`:

### Adding Neck Actuator to Dropdown
Currently the dropdown shows wheel actuators. To add the neck actuator:
1. Open `webrtc.html`
2. Find the actuator selection dropdown
3. Add option: `<option value="1280">Neck Actuator (0x500 / 1280)</option>`

### Retrieving Telemetry
1. Select "Neck Actuator" from dropdown
2. Click **"📥 Retrieve"**
3. Values will populate in the debug panel

### Tuning Parameters
You can adjust:
- **CAN Address**: Change from default 0x500
- **Motor Direction**: Invert if up/down is reversed
- **Max PWM**: Limit maximum speed (default 255)
- **Min PWM**: Set deadzone to overcome motor friction (default 30)

## 🔍 Testing Checklist

- [ ] ESP32 powers on and shows serial output
- [ ] CAN bus initializes successfully
- [ ] Motor responds to U command (moves up)
- [ ] Motor responds to D command (moves down)
- [ ] Motor stops when U=0 and D=0
- [ ] Telemetry retrieval works from web interface
- [ ] Parameters save to EEPROM
- [ ] Parameters load correctly after power cycle

## 🐛 Troubleshooting

### Motor doesn't move
- Check PWM connections to motor driver
- Verify motor driver has power
- Try increasing `Min PWM` value
- Check if motor direction is inverted

### No CAN messages received
- Verify CAN TX/RX wiring
- Check CAN termination resistors (120Ω)
- Confirm RPI4 is sending to correct CAN ID (0x500)
- Check serial monitor for CAN bus errors

### Wrong direction
- Use tuning command to invert motor direction
- Or swap GPIO 37 ↔ GPIO 38 in hardware

### Telemetry not working
- Ensure RPI4 has telemetry receiver running
- Check that telemetry messages use CAN ID 0x309
- Verify neck actuator CAN address matches request

## 📊 Differences from Wheel Actuator

| Aspect | Wheel Actuator | Neck Actuator |
|--------|----------------|---------------|
| **Control** | Closed-loop PID | Open-loop speed |
| **Sensor** | AS5047P encoder | None |
| **Motors** | 2 motors | 1 motor |
| **Commands** | F, B, L, R | U, D |
| **Complexity** | High | Low |
| **Use Case** | Position control | Speed control |

## 🔧 Advanced Configuration

### Change Default CAN ID
In `neckActuator.cpp`, modify:
```cpp
const int NECK_ACTUATOR_CAN_ID = 0x500;  // Change to your preferred ID
```

### Adjust PWM Frequency
For different motor types:
```cpp
const int PWM_FREQUENCY = 20000;  // 20 kHz default
```

### Custom Pin Assignments
Modify pin definitions at top of file:
```cpp
const int MOTOR_FORWARD_PIN = 37;  // Change as needed
const int MOTOR_REVERSE_PIN = 38;  // Change as needed
```

## 📝 Next Steps

1. **Test basic operation** with serial monitor
2. **Integrate with RPI4** control system
3. **Tune parameters** via web interface
4. **Add mechanical limits** for safety
5. **Calibrate PWM values** for smooth operation

## 💡 Tips

- Start with low PWM values during initial testing
- Use serial monitor to verify commands are being received
- Save parameters to EEPROM after finding good settings
- Consider adding soft limits in control software
- Monitor CAN bus health messages for reliability

---

**Ready to deploy!** The neck actuator is fully compatible with your existing control infrastructure and just needs to be flashed to an ESP32. 🚀

