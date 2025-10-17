#!/usr/bin/env python3
"""
Test script to debug telemetry forwarding issue
This script sends a test telemetry message directly to the WebSocket
to verify the web interface can receive and process it.
"""

import asyncio
import json
import websockets
import sys

async def test_telemetry():
    uri = "ws://64.225.55.176:8080"
    
    print(f"🔗 Connecting to WebSocket: {uri}")
    
    try:
        async with websockets.connect(uri, ping_interval=30, ping_timeout=30) as ws:
            print("✅ WebSocket connected")
            
            # Send a test telemetry message
            test_telemetry = {
                'type': 'telemetry',
                'data': {
                    'address': 512,
                    'motor_dir': False,
                    'sensor_dir': False,
                    'max_pwm': 255,
                    'min_pwm': 30,
                    'setpoint': 0.0,
                    'p_gain': 5.0
                }
            }
            
            print(f"📤 Sending test telemetry: {json.dumps(test_telemetry, indent=2)}")
            await ws.send(json.dumps(test_telemetry))
            print("✅ Test telemetry sent")
            
            # Wait a moment for the message to be processed
            await asyncio.sleep(2)
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    print("🧪 Telemetry Test Script")
    print("=" * 50)
    print("This script sends a test telemetry message to the WebSocket")
    print("to verify the web interface can receive and process it.")
    print("=" * 50)
    
    try:
        asyncio.run(test_telemetry())
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Test failed: {e}")
    
    print("🏁 Test complete")
