# Wheel Actuator Tuning System Guide

This guide explains the tuning system for wheel actuators, allowing real-time parameter adjustment via CAN bus.

## System Architecture

```
┌─────────────────┐         ┌──────────────┐         ┌───────────────────┐
│  Web Browser    │  WebSocket  │   RPI4       │   CAN    │  ESP32 Wheel      │
│  (webrtc.html)  │ ────────> │  (rpi4.py)   │ ───────> │  Actuator         │
│  Debug Panel    │         │              │         │  (wheelActuator)  │
└─────────────────┘         └──────────────┘         └───────────────────┘
```

## Components Updated

### 1. **wheelActuator.cpp** (ESP32 Firmware)
   - Added tunable parameter storage
   - Added CAN message handlers for tuning commands
   - Applied tuning parameters to motor control and sensor reading

### 2. **rpi4.py** (RPI4 Bridge)
   - Added tuning command definitions
   - Added `send_tuning_command()` function
   - Updated command parser to handle tuning commands

### 3. **webrtc.html** (Web Debug Panel) ⭐ PRIMARY INTERFACE
   - Integrated debug panel in WebRTC interface
   - UI controls for all tuning parameters
   - Real-time feedback and control string display
   - Toggle switches for boolean parameters
   - Send buttons for each tuning command

### 4. **tuning_debug.py** (Alternative CLI Tool)
   - Standalone command-line tool for sending tuning commands
   - Interactive terminal interface
   - Useful for automated scripting

## Available Tuning Commands

| Command          | CAN ID | Description                              | Value Type    | Range      |
|------------------|--------|------------------------------------------|---------------|------------|
| tune_address     | 0x300  | Change CAN address                       | uint16        | 0x000-0x7FF|
| tune_setpoint    | 0x301  | Set actuator zero position offset        | float         | degrees    |
| tune_motor_dir   | 0x302  | Switch motor pins direction              | bool          | 0 or 1     |
| tune_sensor_dir  | 0x303  | Switch sensor count direction            | bool          | 0 or 1     |
| tune_p           | 0x304  | Update P gain value                      | float         | > 0        |
| tune_max_pwm     | 0x305  | Update maximum PWM value                 | uint8         | 0-255      |
| tune_min_pwm     | 0x306  | Update minimum PWM (deadzone)            | uint8         | 0-255      |
| tune_save        | 0x307  | Save parameters to EEPROM                | uint8         | 1          |

## Usage Instructions

### Quick Start (Web Interface - Recommended)

1. **Ensure RPI4 is running** with `rpi4.py` active
2. **Open the WebRTC interface** in your browser:
   ```
   Open: webrtc.html
   ```
3. **Access the Debug Panel:**
   - The debug panel is visible on the right side of the screen
   - Click "Hide" to collapse it, "Show" to expand it
   - Shows current control strings being sent in real-time

4. **Send Tuning Commands:**
   - Enter values in the input fields
   - Toggle switches for direction inversions
   - Click the "Send" button for each parameter
   - Button will flash green with "✓ Sent" confirmation
   - Command appears in the control string display

### Using the Web Debug Panel

#### 📡 Retrieving Current Values (NEW!)

Before making changes, you can retrieve the current parameter values from any actuator:

1. **Select the target actuator** from the dropdown:
   - Wheel Actuator 1 (0x200 / 512)
   - Wheel Actuator 2 (0x300 / 768)
   - Wheel Actuator 3 (0x400 / 1024)
   - All Actuators (Broadcast) - for discovery

2. **Click the "📥 Retrieve" button**
   - Status shows "⏳ Requesting telemetry..."
   - Wait ~0.5 seconds for response
   - Status updates to "✅ Loaded values from actuator 0xXXX"

3. **All fields populate automatically:**
   - Input fields flash green briefly
   - Values loaded directly from actuator
   - Toggle switches update to match current state

4. **Make changes and send** as needed

**Tip:** Use broadcast mode (0xFFFF) if you don't know the actuator's address!

---

#### 🎯 CAN Address
- Enter the device CAN address in decimal (e.g., 512 for 0x200)
- Click "Send" to update

#### 📐 Actuator Setpoint
- Enter the zero position offset in degrees (can be negative)
- Click "Send" to update

#### ⬅️➡️ Motor Direction
- Toggle switch: OFF = Normal, ON = Inverted
- Click "Send" to apply

#### 🔄 Sensor Direction  
- Toggle switch: OFF = Normal, ON = Inverted
- Click "Send" to apply

#### 📊 P Gain
- Enter proportional gain value (typically 3.0-10.0)
- Click "Send" to update

#### ⚡ PWM Limits
- **Max PWM**: Enter maximum PWM (0-255)
- **Min PWM**: Enter deadzone/minimum PWM (0-255)
- Click "Send" for each

#### 💾 Save Parameters
- Click "Save Parameters" button
- Current values are saved to ESP32 flash memory (Preferences)
- Parameters automatically load on next boot
- Saved values include:
  - CAN Address
  - Actuator Setpoint
  - Motor/Sensor Direction flags
  - P Gain
  - Max/Min PWM limits

---

### Alternative: Command-Line Tool

For scripting or when GUI is not available:

1. **Run the CLI tool:**
   ```bash
   # On Windows
   run_tuning_debug.bat
   
   # On Linux/Mac
   python3 tuning_debug.py
   ```

2. **Enter commands:**
   ```
   tuning> tune_p=7.5
   tuning> tune_max_pwm=200
   tuning> tune_motor_dir=1
   ```

## CAN Message Format

### Drive Commands (Existing)
- **CAN ID:** 0x200 (DRIVE_SYSTEM_ID)
- **Payload:** 8 bytes [f, b, l, r, u, d, l2, r2]

### Tuning Commands (New)

#### Set Address (0x300)
- **Payload:** 2 bytes [high_byte, low_byte] (big-endian uint16)
- **Example:** 0x200 = [0x02, 0x00]

#### Set Setpoint (0x301)
- **Payload:** 4 bytes (little-endian float)
- **Example:** 2.5° = [0x00, 0x00, 0x20, 0x40]

#### Motor Direction (0x302)
- **Payload:** 1 byte (0=normal, 1=inverted)

#### Sensor Direction (0x303)
- **Payload:** 1 byte (0=normal, 1=inverted)

#### Update P Gain (0x304)
- **Payload:** 4 bytes (little-endian float)

#### Max PWM (0x305)
- **Payload:** 1 byte (0-255)

#### Min PWM (0x306)
- **Payload:** 1 byte (0-255)

#### Save Parameters (0x307)
- **Payload:** 1 byte (typically 1)

## Troubleshooting

### Connection Issues
- Verify RPI4 IP address in `tuning_debug.py` (default: `ws://64.225.55.176:8080`)
- Ensure `rpi4.py` is running on the RPI4
- Check network connectivity

### Commands Not Working
- Verify CAN interface is up: `ip link show can0`
- Check RPI4 console for error messages
- Ensure ESP32 is powered and CAN bus is properly connected

### Motor Behaving Erratically
- Check if motor direction is inverted: `tune_motor_dir=0` or `tune_motor_dir=1`
- Verify sensor direction: `tune_sensor_dir=0` or `tune_sensor_dir=1`
- Adjust P gain - too high causes oscillation, too low causes sluggish response

### PWM Not Responding
- Ensure `tune_min_pwm` is less than `tune_max_pwm`
- Typical values: min=30-40, max=200-255
- Deadzone (min_pwm) compensates for motor friction

## Parameter Tuning Tips

### P Gain Tuning
1. Start with P=5.0
2. Increase gradually if response is too slow
3. Decrease if motor oscillates or overshoots
4. Typical range: 3.0 - 10.0

### PWM Tuning
1. **Max PWM:** Start at 255, reduce if motor gets too hot or violent
2. **Min PWM (Deadzone):** 
   - Start at 30
   - Increase if motor doesn't start moving at low speeds
   - Should be just above the motor's static friction threshold

### Direction Tuning
1. Test motor response with small commands
2. If motor moves opposite to command, toggle `tune_motor_dir`
3. If position readings are inverted, toggle `tune_sensor_dir`

## Serial Monitor Output

When tuning commands are received, the ESP32 will output debug information:

```
Tuning command received: 0x304
-> P value updated to: 7.500

Tuning command received: 0x305
-> Max PWM set to: 220

Tuning command received: 0x302
-> Motor direction inverted: YES
```

## Future Enhancements

- [x] EEPROM storage for persistent parameters ✅ **IMPLEMENTED**
- [ ] Parameter profiles for different operating modes
- [ ] Factory reset command to restore defaults
- [ ] Auto-tuning algorithm
- [ ] Real-time parameter monitoring and graphing
- [ ] Multiple actuator addressing support

## Safety Notes

⚠️ **Important:**
- Test with low PWM values first
- Ensure adequate power supply for motors
- Monitor motor temperature during tuning
- Keep emergency stop accessible
- Start with conservative P gains

## Technical Details

### ESP32 Implementation
- Tuning parameters stored in global variables
- Applied in real-time to control loops
- Direction flags applied at sensor/motor level
- PWM limits enforced in `setMotorSpeed()`

### Communication Protocol
- WebSocket from PC to RPI4
- CAN bus from RPI4 to ESP32
- Command string format: `key=value;key=value;...`
- Non-blocking send/receive operations

## Support

For issues or questions:
1. Check this guide first
2. Review console output for error messages
3. Verify CAN bus connections and termination
4. Test with the provided example commands

---

*Last Updated: October 2025*
*Version: 1.0*

