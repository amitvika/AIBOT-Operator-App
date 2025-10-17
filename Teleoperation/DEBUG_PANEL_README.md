# Web Debug Panel for Wheel Actuator Tuning

## Overview

The WebRTC interface now includes an integrated debug panel for real-time tuning of wheel actuator parameters. This replaces the need for a separate command-line tool and provides an intuitive graphical interface.

## Features

✨ **Real-Time Control String Display**
- Shows exactly what commands are being sent
- Timestamps for each transmission
- Updates every 50ms with current control state

🎛️ **Comprehensive Tuning Controls**
- CAN Address adjustment
- Actuator zero position offset
- Motor direction inversion (toggle switch)
- Sensor direction inversion (toggle switch)
- P gain adjustment
- PWM limits (max and min)
- Save parameters command

✅ **Visual Feedback**
- Send buttons flash green on successful transmission
- Toggle switches show current state (Normal/Inverted)
- Real-time control string updates

🔄 **Collapsible Panel**
- Hide/Show button to maximize video space
- Floating toggle button when hidden
- Smooth transitions

## Interface Layout

```
┌─────────────────────────────────────────────────┐
│  🎥 Video Stream                      [🔧 Debug]│
│                                                  │
│                                    ┌─────────────┤
│                                    │ 📡 Control  │
│                                    │ String      │
│                                    ├─────────────┤
│                                    │ ⚙️ Tuning   │
│                                    │             │
│                                    │ • Address   │
│                                    │ • Setpoint  │
│                                    │ • Motor Dir │
│                                    │ • Sensor Dir│
│                                    │ • P Gain    │
│                                    │ • Max PWM   │
│                                    │ • Min PWM   │
│                                    │ • Save      │
│                                    └─────────────┘
└──────────────────────────────────────────────────┘
```

## Usage Guide

### Opening the Interface

1. Ensure RPI4 is running with `rpi4.py`
2. Open `webrtc.html` in a modern web browser
3. The debug panel appears on the right side

### Sending Tuning Commands

#### For Numeric Values (Address, Setpoint, P Gain, PWM):
1. Enter the desired value in the input field
2. Click the "Send" button
3. Button flashes green and shows "✓ Sent"
4. Command appears in the control string display

#### For Direction Toggles (Motor/Sensor):
1. Click the toggle switch to change state
2. Label updates (Normal ↔ Inverted)
3. Click the "Send" button to apply
4. Button flashes green and shows "✓ Sent"

#### Save Parameters:
1. Click the "💾 Save Parameters" button
2. Command is sent to ESP32
3. Note: EEPROM storage is not yet implemented

### Panel Management

**To Hide the Panel:**
- Click "Hide" button in the panel header
- Panel slides away
- Floating "Show Debug Panel" button appears

**To Show the Panel:**
- Click "Show" in collapsed state, or
- Click the floating "Show Debug Panel" button

## Parameter Reference

### CAN Address
- **Format:** Integer (0-2047)
- **Default:** 512 (0x200)
- **Description:** Device address on CAN bus
- **Example:** 512, 768, 1024

### Actuator Setpoint
- **Format:** Float (degrees)
- **Default:** 0.0
- **Description:** Zero position offset for calibration
- **Example:** 2.5, -1.8, 0.0

### Motor Direction
- **Format:** Boolean (Normal/Inverted)
- **Default:** Normal
- **Description:** Reverses motor forward/reverse
- **When to Use:** Motor moves opposite to command

### Sensor Direction
- **Format:** Boolean (Normal/Inverted)
- **Default:** Normal
- **Description:** Reverses sensor reading direction
- **When to Use:** Position readings are backwards

### P Gain
- **Format:** Float
- **Default:** 5.0
- **Range:** 3.0-10.0 (typical)
- **Description:** Proportional gain for position control
- **Tuning:**
  - Too low → Slow response
  - Too high → Oscillation

### Maximum PWM
- **Format:** Integer (0-255)
- **Default:** 255
- **Description:** Upper limit for motor power
- **Tuning:** Reduce if motor overheats

### Minimum PWM
- **Format:** Integer (0-255)
- **Default:** 30
- **Description:** Deadzone compensation
- **Tuning:** Just above static friction threshold

## Real-Time Feedback

### Control String Display
Located at the top of the debug panel, this shows:
- Timestamp of last update
- Complete control string being transmitted
- Updates automatically every 50ms
- Shows both regular controls and tuning commands

### Button Feedback
When you click "Send":
1. Command is transmitted via WebSocket
2. Button background turns green
3. Text changes to "✓ Sent"
4. Returns to normal after 1 second

### Console Logging
Open browser console (F12) to see:
```
✅ Sent tuning command: tune_p=7.5
DEBUG: Sent control string: tune_p=7.5
```

## Troubleshooting

### "WebSocket not connected!" alert
**Problem:** No connection to RPI4  
**Solution:**
- Verify RPI4 is powered on
- Check that `rpi4.py` is running
- Confirm WebSocket URI (default: `ws://64.225.55.176:8080`)
- Check network connectivity

### Button doesn't flash green
**Problem:** Command may not have been sent  
**Solution:**
- Check browser console for errors
- Verify WebSocket connection status
- Ensure input value is valid

### Input validation errors
**Problem:** "Please enter a valid number"  
**Solution:**
- Numeric fields require valid numbers
- Check for typos or non-numeric characters
- Decimals use period (.) not comma (,)

### Tuning command has no effect
**Problem:** ESP32 not responding  
**Solution:**
- Check RPI4 console for CAN errors
- Verify CAN bus is connected and terminated
- Check ESP32 serial monitor for received commands
- Confirm CAN address matches

## Best Practices

### 🔒 Safety First
- Start with conservative values
- Test with low PWM first
- Monitor motor temperature
- Have emergency stop ready

### 📝 Documentation
- Record working parameters
- Note changes and their effects
- Take screenshots of successful configs

### 🧪 Systematic Testing
1. Change one parameter at a time
2. Test thoroughly before next change
3. Return to known-good config if issues arise

### 🎯 Tuning Process
1. **Verify directions first**
   - Test motor direction
   - Check sensor direction
   - Fix any inversions

2. **Adjust PWM limits**
   - Test minimum PWM for smooth starts
   - Adjust maximum if needed

3. **Tune P gain**
   - Start at default (5.0)
   - Increase for faster response
   - Decrease if oscillating

4. **Fine-tune setpoint**
   - Adjust zero position if needed
   - Test full range of motion

## Technical Details

### Communication Flow
```
User Input → sendTuningCommand() → WebSocket → RPI4 → CAN Bus → ESP32
```

### Message Format
```javascript
{
  "type": "control",
  "controlString": "tune_p=7.5"
}
```

### Update Rate
- Regular control strings: 20 Hz (50ms intervals)
- Tuning commands: On-demand (user triggered)

### Browser Compatibility
- Chrome/Edge: Full support ✅
- Firefox: Full support ✅
- Safari: Full support ✅
- Mobile browsers: Works but cramped on small screens

## Keyboard Shortcuts

Currently no keyboard shortcuts implemented, but you can:
- Tab through input fields
- Enter to focus next field
- Use browser zoom (Ctrl + / Ctrl -)

## Future Enhancements

Potential improvements:
- [ ] Preset profiles for common configurations
- [ ] Parameter import/export
- [ ] Real-time parameter monitoring graphs
- [ ] Multi-actuator addressing
- [ ] Keyboard shortcuts for sending
- [ ] Parameter history/undo
- [ ] Touch-optimized mobile layout

## Related Documentation

- **TUNING_GUIDE.md** - Comprehensive tuning system documentation
- **CAN Messaging Documentation** - CAN protocol details
- **wheelActuator.cpp** - ESP32 firmware implementation
- **rpi4.py** - RPI4 bridge implementation

---

**Version:** 1.0  
**Last Updated:** October 2025  
**Author:** Wheel Actuator Control System Team

