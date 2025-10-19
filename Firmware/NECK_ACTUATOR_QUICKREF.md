# Neck Actuator - Quick Reference Card

## 📋 Quick Facts
- **File**: `neckActuator.cpp`
- **CAN ID**: `0x500` (1280 decimal)
- **Control**: Open-loop speed control
- **Commands**: U (up, byte 4) and D (down, byte 5)
- **Motor**: Single bidirectional motor

## 🔌 Pin Connections
```
CAN:  TX=48, RX=34, STBY=47
Motor: FWD=37, REV=38
```

## 🎮 Operation
```
U=100, D=0   → Full speed up
U=50, D=0    → Half speed up
U=0, D=100   → Full speed down
U=0, D=0     → Stop
```

## ⚙️ Default Parameters
```
Max PWM: 255
Min PWM: 30
Motor Dir: Normal (not inverted)
```

## 🔧 Tuning via Web UI
1. Select "Neck Actuator (0x500 / 1280)" from dropdown
2. Click "📥 Retrieve" to load current values
3. Adjust parameters as needed
4. Click "📤 Send" to apply
5. Click "💾 Save All" to persist

## 📊 Telemetry Format
```
CAN ID: 0x309 (Single message, auto-detected)
Bytes 0-1: Address
Byte 2: Motor direction (0=normal, 1=inverted)
Byte 3: Reserved (0)
Bytes 4-5: Max/Min PWM
Bytes 6-7: Reserved (0)
```

## 🚀 Upload & Test
```bash
# Upload firmware
pio run --target upload

# Serial monitor
pio device monitor -b 115200

# Expected output
--- Neck Actuator CAN Controller (Listening for ID: 0x500) ---
CAN Bus initialized successfully.
Motor PWM outputs configured.
```

## 🐛 Quick Troubleshooting
| Problem | Quick Fix |
|---------|-----------|
| No movement | Increase Min PWM to 50 |
| Wrong direction | Invert motor direction via UI |
| No CAN | Check TX/RX wiring, verify 120Ω termination |
| Too fast | Reduce Max PWM to 200 |
| Too slow | Check Min PWM is not too high |

## 📚 Documentation
- **Full Reference**: `NECK_ACTUATOR_README.md`
- **Setup Guide**: `NECK_ACTUATOR_SETUP.md`
- **Integration**: `NECK_ACTUATOR_INTEGRATION_SUMMARY.md`

