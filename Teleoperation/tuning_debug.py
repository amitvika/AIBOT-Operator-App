#!/usr/bin/env python3
"""
Tuning Debug Tool for Wheel Actuators
───────────────────────────────────────
This tool allows you to send tuning commands to wheel actuators via the RPI4.
It connects to the RPI4 WebSocket server and sends tuning commands in the
same format as regular control commands.

Usage:
    python tuning_debug.py
    
Commands available:
    - tune_address=<value>      : Set CAN address (hex, e.g., 0x200)
    - tune_setpoint=<value>     : Set actuator zero position offset (degrees)
    - tune_motor_dir=<0|1>      : Switch motor direction (0=normal, 1=inverted)
    - tune_sensor_dir=<0|1>     : Switch sensor direction (0=normal, 1=inverted)
    - tune_p=<value>            : Update P gain value (float)
    - tune_max_pwm=<value>      : Set maximum PWM (0-255)
    - tune_min_pwm=<value>      : Set minimum PWM/deadzone (0-255)
    - tune_save=1               : Save parameters to EEPROM
    - quit                      : Exit the program
"""

import asyncio
import json
import sys
import websockets
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError

# Configuration
RPI4_WEBSOCKET_URI = "ws://64.225.55.176:8080"  # Update with your RPI4 IP if different

# Tuning command definitions (must match rpi4.py)
TUNING_COMMANDS = {
    "tune_address": "Set CAN address (hex, e.g., 512 for 0x200)",
    "tune_setpoint": "Set actuator zero position offset (degrees, float)",
    "tune_motor_dir": "Switch motor direction (0=normal, 1=inverted)",
    "tune_sensor_dir": "Switch sensor direction (0=normal, 1=inverted)",
    "tune_p": "Update P gain value (float)",
    "tune_max_pwm": "Set maximum PWM (0-255)",
    "tune_min_pwm": "Set minimum PWM/deadzone (0-255)",
    "tune_save": "Save parameters to EEPROM (value=1)",
}

def print_help():
    """Print available commands"""
    print("\n" + "="*60)
    print("Tuning Debug Tool - Available Commands")
    print("="*60)
    for cmd, desc in TUNING_COMMANDS.items():
        print(f"  {cmd:<20} : {desc}")
    print("\n  quit                 : Exit the program")
    print("="*60)
    print("\nFormat: command=value (e.g., tune_p=7.5)")
    print("You can send multiple commands separated by semicolons")
    print("Example: tune_p=5.0;tune_max_pwm=200\n")


class TuningDebugClient:
    def __init__(self, uri):
        self.uri = uri
        self.ws = None
        self.connected = False
        
    async def connect(self):
        """Connect to the WebSocket server"""
        try:
            print(f"Connecting to {self.uri}...")
            self.ws = await websockets.connect(self.uri, ping_interval=30, ping_timeout=30)
            self.connected = True
            print(f"✅ Connected to RPI4 at {self.uri}\n")
            return True
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    async def send_command(self, command_string):
        """Send a control string with tuning commands"""
        if not self.connected or not self.ws:
            print("❌ Not connected to RPI4")
            return False
        
        try:
            # Send as a control message (same format as regular control commands)
            message = {
                "type": "control",
                "controlString": command_string
            }
            await self.ws.send(json.dumps(message))
            print(f"✅ Sent: {command_string}")
            return True
        except Exception as e:
            print(f"❌ Send failed: {e}")
            self.connected = False
            return False
    
    async def close(self):
        """Close the WebSocket connection"""
        if self.ws:
            await self.ws.close()
            self.connected = False
            print("Connection closed.")
    
    async def interactive_mode(self):
        """Run interactive command mode"""
        print_help()
        
        while True:
            try:
                # Get user input (in async context, we need to use a thread)
                command = await asyncio.get_event_loop().run_in_executor(
                    None, 
                    input, 
                    "tuning> "
                )
                
                command = command.strip()
                
                if not command:
                    continue
                
                if command.lower() in ['quit', 'exit', 'q']:
                    print("Exiting...")
                    break
                
                if command.lower() in ['help', 'h', '?']:
                    print_help()
                    continue
                
                # Validate command format
                if '=' not in command:
                    print("❌ Invalid format. Use: command=value")
                    print("   Type 'help' for available commands")
                    continue
                
                # Parse and validate commands
                valid = True
                commands = command.split(';')
                for cmd in commands:
                    if '=' not in cmd:
                        continue
                    key, _ = cmd.split('=', 1)
                    if key not in TUNING_COMMANDS:
                        print(f"❌ Unknown command: {key}")
                        print("   Type 'help' for available commands")
                        valid = False
                        break
                
                if not valid:
                    continue
                
                # Send the command
                await self.send_command(command)
                
            except KeyboardInterrupt:
                print("\n\nInterrupted. Type 'quit' to exit.")
            except Exception as e:
                print(f"❌ Error: {e}")


async def main():
    """Main entry point"""
    print("="*60)
    print("  Wheel Actuator Tuning Debug Tool")
    print("="*60)
    print(f"Target: {RPI4_WEBSOCKET_URI}\n")
    
    client = TuningDebugClient(RPI4_WEBSOCKET_URI)
    
    if not await client.connect():
        print("\n❌ Failed to connect. Please check:")
        print("   1. RPI4 is running and rpi4.py script is active")
        print("   2. WebSocket URI is correct")
        print("   3. Network connectivity")
        sys.exit(1)
    
    try:
        await client.interactive_mode()
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        await client.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)

