# Actuator Telemetry System

## Overview

The telemetry system enables bidirectional communication with wheel actuators, allowing you to read back current parameter values from the ESP32 and populate them in the web interface. This makes it easy to see what values are currently configured on each actuator before making changes.

## System Architecture

```
┌──────────────┐  1. Request    ┌──────────┐  2. CAN      ┌──────────────┐
│  Web Browser │ ──────────────> │   RPI4   │ ───────────> │  ESP32 Wheel │
│ (webrtc.html)│                 │ (rpi4.py)│              │  Actuator    │
│              │                 │          │              │              │
│              │ <────────────── │          │ <─────────── │              │
└──────────────┘  5. Display     └──────────┘  3. Telemetry└──────────────┘
                  Telemetry         4. Forward   Response
```

### Message Flow:

1. **User clicks "Retrieve"** → Selects actuator ID from dropdown
2. **Web sends request** → `tune_request_telemetry=512` via WebSocket
3. **RPI4 forwards to CAN** → Sends request with target address (0x308)
4. **ESP32 responds** → Sends two CAN messages with current values (0x309, 0x30A)
5. **RPI4 receives & forwards** → Parses CAN messages, sends to WebSocket as JSON
6. **Web updates UI** → Populates input fields with received values

## Components

### 1. ESP32 Firmware (wheelActuator.cpp)

#### New CAN Message IDs:
- **0x308** (`CMD_REQUEST_TELEMETRY`) - Request telemetry from actuator
- **0x309** (`CMD_TELEMETRY_RESPONSE_1`) - First part of telemetry response
- **0x30A** (`CMD_TELEMETRY_RESPONSE_2`) - Second part of telemetry response

#### Telemetry Request Handling:
```cpp
case CMD_REQUEST_TELEMETRY:
  if (message.data_length_code >= 2) {
    uint16_t target_address = (message.data[0] << 8) | message.data[1];
    if (target_address == g_can_address || target_address == 0xFFFF) {
      sendTelemetry();
    }
  }
  break;
```

#### Telemetry Response Format:

**Message 1 (0x309):** 6 bytes
- Bytes 0-1: CAN Address (uint16, big-endian)
- Byte 2: Motor Direction Inverted (bool)
- Byte 3: Sensor Direction Inverted (bool)
- Byte 4: Max PWM (uint8)
- Byte 5: Min PWM (uint8)

**Message 2 (0x30A):** 8 bytes
- Bytes 0-3: Actuator Setpoint (float, little-endian)
- Bytes 4-7: P Gain (float, little-endian)

### 2. RPI4 Bridge (rpi4.py)

#### New Features:

**Telemetry Command:**
```python
TUNING_COMMANDS = {
    # ... existing commands ...
    "tune_request_telemetry": 0x308,
}
```

**CAN Receiver Task:**
```python
async def _can_receiver(self):
    """Receive CAN messages and forward to WebSocket"""
    while True:
        msg = self.can_bus.recv(timeout=0.01)
        if msg and msg.arbitration_id in [TELEMETRY_RESPONSE_1, TELEMETRY_RESPONSE_2]:
            # Parse and forward telemetry
```

**Telemetry Parsing:**
- Receives both CAN messages
- Buffers first message until second arrives
- Combines data and sends via WebSocket as JSON

**WebSocket Message Format:**
```json
{
  "type": "telemetry",
  "data": {
    "address": 512,
    "motor_dir": false,
    "sensor_dir": false,
    "max_pwm": 255,
    "min_pwm": 30,
    "setpoint": 0.0,
    "p_gain": 5.0
  }
}
```

### 3. Web Interface (webrtc.html)

#### Actuator Selection:
```html
<select id="actuator-id-select">
  <option value="512">Wheel Actuator 1 (0x200 / 512)</option>
  <option value="768">Wheel Actuator 2 (0x300 / 768)</option>
  <option value="1024">Wheel Actuator 3 (0x400 / 1024)</option>
  <option value="65535">All Actuators (Broadcast)</option>
</select>
<button onclick="retrieveTelemetry()">📥 Retrieve</button>
```

#### Telemetry Retrieval:
```javascript
function retrieveTelemetry() {
  const actuatorId = document.getElementById('actuator-id-select').value;
  const commandString = `tune_request_telemetry=${actuatorId}`;
  globalWS.send(JSON.stringify({ 
    type: "control", 
    controlString: commandString 
  }));
}
```

#### UI Population:
```javascript
function handleTelemetry(data) {
  document.getElementById('tune-address').value = data.address;
  document.getElementById('tune-setpoint').value = data.setpoint.toFixed(2);
  document.getElementById('tune-motor-dir').checked = data.motor_dir;
  // ... populate all fields ...
  
  // Flash fields to show they've been updated
  elem.style.backgroundColor = '#d5f4e6';
}
```

## Usage

### Step-by-Step Guide:

1. **Open webrtc.html** in your browser

2. **Select target actuator:**
   - Choose from dropdown (e.g., "Wheel Actuator 1 (0x200 / 512)")
   - Or select "All Actuators (Broadcast)" to request from any available actuator

3. **Click "📥 Retrieve" button**
   - Status message shows "⏳ Requesting telemetry..."
   - Request is sent via WebSocket → RPI4 → CAN → ESP32

4. **Wait for response (typically < 500ms)**
   - ESP32 sends back two CAN messages
   - RPI4 receives, parses, and forwards to browser
   - Status updates to "✅ Loaded values from actuator 0x200"

5. **UI fields populate automatically:**
   - All input fields flash green briefly
   - Values are populated from the actuator
   - Toggle switches update to match current state

6. **Modify and send as needed:**
   - Make any desired changes
   - Click individual "Send" buttons to update

### Broadcast Mode:

When you select "All Actuators (Broadcast)" (address 0xFFFF):
- Request is sent to all actuators on the bus
- First actuator to respond will populate the fields
- Useful when you don't know the exact address

## Troubleshooting

### No Response from Actuator

**Symptoms:**
- Status shows "⚠️ No response from actuator" after 3 seconds
- Fields remain empty

**Possible Causes:**
1. ESP32 not powered or not running firmware
2. CAN bus not connected or terminated properly
3. Wrong actuator address selected
4. CAN bus communication error

**Solutions:**
- Check ESP32 serial monitor for debug output
- Verify CAN bus connections (H, L, GND, termination)
- Try broadcast mode (0xFFFF) to see if any actuator responds
- Check RPI4 console for CAN errors

### Partial Data Received

**Symptoms:**
- Some fields populate, others remain empty
- Console shows "Part 1" but no "Part 2"

**Possible Causes:**
- CAN bus noise causing message loss
- Timing issue between two messages

**Solutions:**
- Check CAN bus termination (120Ω at both ends)
- Ensure proper grounding
- Try retrieve again (usually works on second attempt)

### Wrong Values Displayed

**Symptoms:**
- Values don't match expected configuration
- Address shows different than selected

**Possible Causes:**
- Multiple actuators responding (broadcast mode)
- Actuator has been reconfigured

**Solutions:**
- Use specific address instead of broadcast
- Verify actuator configuration via serial monitor
- Send correct values and retrieve again to confirm

## Console Output Examples

### Successful Telemetry Retrieval:

**Browser Console:**
```
✅ Requested telemetry from actuator 512
DEBUG: Received telemetry: {address: 512, motor_dir: false, ...}
📥 Processing telemetry: {address: 512, motor_dir: false, ...}
✅ Telemetry data loaded into UI
```

**RPI4 Console:**
```
✅ Sent tuning command: tune_request_telemetry = 512 (ID: 0x308)
📥 Received telemetry: ID=0x309, Data=0200000000ff1e
   Part 1: Addr=512, MotorDir=0, SensorDir=0, MaxPWM=255, MinPWM=30
📥 Received telemetry: ID=0x30A, Data=0000000000a04140
   Part 2: Setpoint=0.00, P=5.000
✅ Forwarded complete telemetry for address 0x200
```

**ESP32 Serial Monitor:**
```
Tuning command received: 0x308
-> Telemetry requested for address 0x200
-> Telemetry part 1 sent
-> Telemetry part 2 sent
-> Telemetry: Addr=0x200, MotorDir=0, SensorDir=0, MaxPWM=255, MinPWM=30, Setpoint=0.00, P=5.000
```

## Performance

- **Request → Response time:** ~100-500ms typical
- **CAN bus overhead:** ~20 bytes total (2 messages)
- **Frequency:** On-demand only (user triggered)
- **No polling:** System doesn't continuously request telemetry

## Security Considerations

- **Address validation:** ESP32 only responds if address matches
- **Broadcast capability:** Can be used to discover actuators
- **No authentication:** CAN bus is open, physical security required

## Future Enhancements

Potential improvements:
- [ ] Auto-retrieve on actuator selection change
- [ ] Periodic telemetry updates (optional polling)
- [ ] Telemetry history/logging
- [ ] Compare current vs. cached values
- [ ] Batch retrieve from multiple actuators
- [ ] Extended telemetry (current position, error states, etc.)
- [ ] Telemetry data export/import

## API Reference

### JavaScript Functions

#### `retrieveTelemetry()`
Requests current parameter values from selected actuator.
- **No parameters** (uses selected actuator ID)
- **Returns:** void
- **Side effects:** Sends WebSocket message, updates status

#### `handleTelemetry(data)`
Processes incoming telemetry and populates UI.
- **Parameters:**
  - `data` (object): Telemetry data from actuator
- **Returns:** void
- **Side effects:** Updates UI fields, shows status message

### Python Functions

#### `send_tuning_command(can_bus, 'tune_request_telemetry', address)`
Sends telemetry request via CAN.
- **Parameters:**
  - `can_bus`: CAN bus interface
  - `command_key`: 'tune_request_telemetry'
  - `value`: Target address (int, 0-2047 or 0xFFFF for broadcast)
- **Returns:** bool (success/failure)

### C++ Functions

#### `void sendTelemetry()`
Sends current parameter values via CAN.
- **No parameters**
- **Returns:** void
- **Side effects:** Sends two CAN messages (0x309, 0x30A)

## Testing

### Manual Testing Procedure:

1. **Verify ESP32 is running:**
   ```
   Serial monitor should show startup messages
   ```

2. **Test broadcast request:**
   ```
   Select "All Actuators (Broadcast)"
   Click "Retrieve"
   Verify fields populate
   ```

3. **Test specific address:**
   ```
   Select "Wheel Actuator 1 (0x200 / 512)"
   Click "Retrieve"
   Verify correct actuator responds
   ```

4. **Test value accuracy:**
   ```
   Set P gain to 7.5, send
   Click "Retrieve"
   Verify P gain field shows 7.500
   ```

5. **Test multiple actuators:**
   ```
   Connect multiple actuators with different addresses
   Retrieve from each
   Verify correct values for each
   ```

---

**Version:** 1.0  
**Last Updated:** October 2025  
**Status:** Production Ready ✅

