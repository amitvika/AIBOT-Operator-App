#!/usr/bin/env python3
"""
vidcan_merged.py (Real-Time Final Version w/ 8-byte Payload)
──────────────────────────────────────────────────────────────
• **UPDATE:** Added l2 and r2 values to the drive command. The CAN
  payload is now a fully-packed 8-byte message containing [f, b, l, r, u, d, l2, r2].
• Contains all previous stability and real-time control fixes.
"""

import argparse
import asyncio
import json
import sys
import threading
import queue
import logging
import os
import can
import struct
import time

import cv2
import numpy as np
import websockets
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError
from aiortc import (
    RTCPeerConnection,
    RTCConfiguration,
    RTCIceServer,
    RTCSessionDescription,
    VideoStreamTrack,
    RTCIceCandidate
)
from av import VideoFrame

# --- CAN BUS INTEGRATION: Configuration ---
LOG_INCOMING_MESSAGES = True
CAN_INTERFACE = 'can0'
DRIVE_SYSTEM_ID = 0x200
COMMAND_TO_CAN_ID_MAP = {
    "h1X": 0x101, "h1Y": 0x102, "h1Z": 0x103, "h1p": 0x104, "h1y": 0x105, "h1r": 0x106, "g1":  0x110,
    "h2X": 0x121, "h2Y": 0x122, "h2Z": 0x123, "h2p": 0x124, "h2y": 0x125, "h2r": 0x126, "g2":  0x130,
}
# **UPDATED**: Added l2 and r2 to define the new 8-byte payload order.
DRIVE_COMMAND_KEYS = ['f', 'b', 'l', 'r', 'u', 'd', 'l2', 'r2']
CAN_SEND_RATE_HZ = 20 # Rate at which we send CAN messages

# --- Tuning Command CAN IDs (must match ESP32) ---
TUNING_BASE_ID = 0x300
TUNING_COMMANDS = {
    "tune_address": TUNING_BASE_ID + 0,     # 0x300
    "tune_setpoint": TUNING_BASE_ID + 1,    # 0x301
    "tune_motor_dir": TUNING_BASE_ID + 2,   # 0x302
    "tune_sensor_dir": TUNING_BASE_ID + 3,  # 0x303
    "tune_p": TUNING_BASE_ID + 4,           # 0x304
    "tune_max_pwm": TUNING_BASE_ID + 5,     # 0x305
    "tune_min_pwm": TUNING_BASE_ID + 6,     # 0x306
    "tune_save": TUNING_BASE_ID + 7,        # 0x307
    "tune_request_telemetry": TUNING_BASE_ID + 8,  # 0x308
}

# Telemetry response IDs
TELEMETRY_RESPONSE_1 = TUNING_BASE_ID + 9   # 0x309
TELEMETRY_RESPONSE_2 = TUNING_BASE_ID + 10  # 0x30A

def setup_can_interface():
    print(f"Bringing up {CAN_INTERFACE}...")
    try:
        os.system(f'sudo ip link set {CAN_INTERFACE} down')
        os.system(f'sudo ip link set {CAN_INTERFACE} type can bitrate 500000')
        os.system(f'sudo ip link set {CAN_INTERFACE} up')
        print(f"✅ {CAN_INTERFACE} is up.")
    except Exception as e:
        print(f"🔴 Error bringing up CAN interface: {e}")
        sys.exit(1)

def send_tuning_command(can_bus, command_key, value):
    """
    Send a tuning command over CAN bus
    
    Args:
        can_bus: CAN bus interface
        command_key: Command key from TUNING_COMMANDS dict
        value: Value to send (int, float, or bool)
    """
    if command_key not in TUNING_COMMANDS:
        print(f"⚠️ Unknown tuning command: {command_key}")
        return False
    
    can_id = TUNING_COMMANDS[command_key]
    
    try:
        # Prepare payload based on command type
        if command_key == "tune_address":
            # 2 bytes: address as uint16
            payload = bytearray(struct.pack('>H', int(value)))
        elif command_key == "tune_setpoint" or command_key == "tune_p":
            # 4 bytes: float value
            payload = bytearray(struct.pack('<f', float(value)))
        elif command_key == "tune_motor_dir" or command_key == "tune_sensor_dir":
            # 1 byte: boolean as uint8
            payload = bytearray([1 if value else 0])
        elif command_key == "tune_max_pwm" or command_key == "tune_min_pwm":
            # 1 byte: PWM value (0-255)
            payload = bytearray([int(value) & 0xFF])
        elif command_key == "tune_save":
            # No payload needed
            payload = bytearray([0])
        elif command_key == "tune_request_telemetry":
            # 2 bytes: target address as uint16 (0xFFFF for broadcast)
            payload = bytearray(struct.pack('>H', int(value)))
        else:
            print(f"⚠️ Unknown tuning command format: {command_key}")
            return False
        
        # Send CAN message
        can_msg = can.Message(
            arbitration_id=can_id,
            data=payload,
            is_extended_id=False
        )
        can_bus.send(can_msg, timeout=0.02)
        print(f"✅ Sent tuning command: {command_key} = {value} (ID: 0x{can_id:03X})")
        return True
        
    except can.CanError as e:
        print(f"⚠️ CAN error sending tuning command: {e}")
        return False
    except Exception as e:
        print(f"⚠️ Error sending tuning command: {e}")
        return False

# --- VIDEO CLASSES (Unchanged) ---
class FrameGrabber:
    def __init__(self, cap):
        self.cap = cap
        self.q = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        t = threading.Thread(target=self._run, daemon=True)
        t.start()
    def _run(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok or frame is None: continue
            try: self.q.get_nowait()
            except queue.Empty: pass
            self.q.put(frame)
    def read(self, timeout=0.5):
        try: return True, self.q.get(timeout=timeout)
        except queue.Empty: return False, None
    def close(self): self._stop.set(); self.cap.release()

class TripleCameraVideoTrack(VideoStreamTrack):
    USB_DEV_LEFT = "/dev/video0"
    USB_DEV_RIGHT = "/dev/video2"
    PIPE_W, PIPE_H = 480, 360
    MAIN_W, MAIN_H = PIPE_H, PIPE_W
    PIP_W, PIP_H = 160, 120
    MARGIN = 10
    def __init__(self, frame_rate=15):
        super().__init__()
        self.frame_rate = frame_rate
        self.frame_period = 1.0 / frame_rate
        self.count = 0
        self.start = asyncio.get_event_loop().time()
        usb_left = cv2.VideoCapture(self.USB_DEV_LEFT, cv2.CAP_V4L2)
        if not usb_left.isOpened(): raise RuntimeError(f"🔴 Cannot open LEFT USB cam at {self.USB_DEV_LEFT}")
        print(f"✅ Left USB cam open ({self.USB_DEV_LEFT})")
        usb_right = cv2.VideoCapture(self.USB_DEV_RIGHT, cv2.CAP_V4L2)
        if not usb_right.isOpened(): raise RuntimeError(f"🔴 Cannot open RIGHT USB cam at {self.USB_DEV_RIGHT}")
        print(f"✅ Right USB cam open ({self.USB_DEV_RIGHT})")
        pipeline = (f"libcamerasrc ! video/x-raw,framerate={frame_rate}/1 ! videoscale ! video/x-raw,width={self.PIPE_W},height={self.PIPE_H} ! videoconvert ! appsink drop=1")
        pi = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not pi.isOpened(): raise RuntimeError("🔴 Cannot open CSI cam")
        print("✅ CSI ribbon cam open")
        self.usb_left_grab = FrameGrabber(usb_left)
        self.usb_right_grab = FrameGrabber(usb_right)
        self.pi_grab = FrameGrabber(pi)
    async def _timestamp(self):
        self.count += 1
        pts = int(self.count * (90000 / self.frame_rate))
        await asyncio.sleep(max(0, self.start + self.count * self.frame_period - asyncio.get_event_loop().time()))
        return pts, 90000
    def _process_frame(self, fp, f_left, f_right):
        fp = cv2.rotate(fp, cv2.ROTATE_90_CLOCKWISE)
        pip_left = cv2.resize(f_left, (self.PIP_W, self.PIP_H), interpolation=cv2.INTER_AREA)
        pip_right = cv2.resize(f_right, (self.PIP_W, self.PIP_H), interpolation=cv2.INTER_AREA)
        fp[self.MARGIN:self.MARGIN+self.PIP_H, self.MARGIN:self.MARGIN+self.PIP_W] = pip_left
        y0_r = self.MARGIN
        x0_r = self.MAIN_W - self.PIP_W - self.MARGIN
        fp[y0_r:y0_r+self.PIP_H, x0_r:x0_r+self.PIP_W] = pip_right
        fp_contiguous = np.ascontiguousarray(fp, dtype=np.uint8)
        rgb = cv2.cvtColor(fp_contiguous, cv2.COLOR_BGR2RGB)
        return VideoFrame.from_ndarray(rgb, format="rgb24")
    async def recv(self):
        pts, time_base = await self._timestamp()
        ok_p, fp = self.pi_grab.read(); ok_l, f_left = self.usb_left_grab.read(); ok_r, f_right = self.usb_right_grab.read()
        if not (ok_p and ok_l and ok_r and fp is not None and f_left is not None and f_right is not None):
            logging.warning("FrameGrabber miss → sending blank frame")
            blank = np.zeros((self.MAIN_H, self.MAIN_W, 3), dtype=np.uint8)
            frame = VideoFrame.from_ndarray(blank, format="rgb24")
            frame.pts, frame.time_base = pts, time_base
            return frame
        try:
            loop = asyncio.get_event_loop()
            frame = await loop.run_in_executor(None, self._process_frame, fp, f_left, f_right)
            frame.pts = pts; frame.time_base = time_base
            return frame
        except Exception as e:
            logging.error(f"Failed to process frame: {e}")
            blank = np.zeros((self.MAIN_H, self.MAIN_W, 3), dtype=np.uint8)
            frame = VideoFrame.from_ndarray(blank, format="rgb24")
            frame.pts, frame.time_base = pts, time_base
            return frame
    def close(self):
        print("Closing all cameras...")
        self.usb_left_grab.close(); self.usb_right_grab.close(); self.pi_grab.close()
        cv2.destroyAllWindows()

# --- Main Application Logic ---
class RobotController:
    def __init__(self, args):
        self.args = args
        self.can_bus = None
        self.latest_control_string = None
        self.pc, self.track = None, None
        self.ws = None  # Store websocket for telemetry forwarding
        self.telemetry_buffer = {}  # Buffer for multi-part telemetry messages

    async def _handle_webrtc_messages(self, ws):
        async for msg in ws:
            data = json.loads(msg)
            msg_type = data.get("type")

            if msg_type == "answer" and self.pc:
                await self.pc.setRemoteDescription(RTCSessionDescription(sdp=data["sdp"], type="answer"))
                print("🔹 Answer set")
            elif msg_type == "candidate" and self.pc:
                try:
                    candidate_data = data.get("candidate")
                    if candidate_data:
                        candidate = RTCIceCandidate(
                            sdpMid=candidate_data.get("sdpMid"),
                            sdpMLineIndex=candidate_data.get("sdpMLineIndex"),
                            candidate=candidate_data.get("candidate")
                        )
                        await self.pc.addIceCandidate(candidate)
                except Exception as e:
                    print(f"⚠️ ICE error: {e}")
            elif msg_type == "control":
                self.latest_control_string = data.get("controlString")
            elif msg_type == "new_offer_request":
                print("🔄 Re-offer request received...")
                await self._create_and_send_offer(ws)

    async def _can_receiver(self):
        """Receive CAN messages (telemetry responses) and forward to WebSocket"""
        while True:
            try:
                # Non-blocking receive with short timeout
                msg = self.can_bus.recv(timeout=0.01)
                
                if msg and msg.arbitration_id in [TELEMETRY_RESPONSE_1, TELEMETRY_RESPONSE_2]:
                    print(f"📥 Received telemetry: ID=0x{msg.arbitration_id:03X}, Data={msg.data.hex()}")
                    
                    # Parse telemetry based on message ID
                    if msg.arbitration_id == TELEMETRY_RESPONSE_1:
                        # Parse: address(2), motor_dir(1), sensor_dir(1), max_pwm(1), min_pwm(1)
                        address = struct.unpack('>H', bytes(msg.data[0:2]))[0]
                        motor_dir = bool(msg.data[2])
                        sensor_dir = bool(msg.data[3])
                        max_pwm = msg.data[4]
                        min_pwm = msg.data[5]
                        
                        # Store in buffer with timestamp
                        self.telemetry_buffer[address] = {
                            'address': address,
                            'motor_dir': motor_dir,
                            'sensor_dir': sensor_dir,
                            'max_pwm': max_pwm,
                            'min_pwm': min_pwm,
                            'timestamp': time.time()
                        }
                        print(f"   Part 1: Addr={address}, MotorDir={motor_dir}, SensorDir={sensor_dir}, MaxPWM={max_pwm}, MinPWM={min_pwm}")
                        
                    elif msg.arbitration_id == TELEMETRY_RESPONSE_2:
                        # Parse: setpoint(4), p_gain(4)
                        setpoint = struct.unpack('<f', bytes(msg.data[0:4]))[0]
                        p_gain = struct.unpack('<f', bytes(msg.data[4:8]))[0]
                        
                        print(f"   Part 2: Setpoint={setpoint:.2f}, P={p_gain:.3f}")
                        
                        # Find matching telemetry part 1 and combine
                        # Since we don't know the address yet, we'll use the most recent one
                        if self.telemetry_buffer:
                            # Get the most recently added address
                            address = list(self.telemetry_buffer.keys())[-1]
                            telemetry_data = self.telemetry_buffer[address].copy()
                            telemetry_data['setpoint'] = setpoint
                            telemetry_data['p_gain'] = p_gain
                            
                            # Remove timestamp before sending
                            if 'timestamp' in telemetry_data:
                                del telemetry_data['timestamp']
                            
                            # Send complete telemetry to WebSocket
                            if self.ws:
                                try:
                                    telemetry_msg = {
                                        'type': 'telemetry',
                                        'data': telemetry_data
                                    }
                                    print(f"📤 Sending telemetry message: {json.dumps(telemetry_msg)}")
                                    await self.ws.send(json.dumps(telemetry_msg))
                                    print(f"✅ Forwarded complete telemetry for address 0x{address:03X}")
                                    print(f"   WebSocket state: open={self.ws.open}, closed={self.ws.closed}")
                                except Exception as e:
                                    print(f"❌ Failed to send telemetry to WebSocket: {e}")
                            else:
                                print("❌ No WebSocket connection available")
                            
                            # Clear from buffer
                            del self.telemetry_buffer[address]
                        else:
                            print("⚠️ No buffered telemetry part 1 found for part 2")
                
                await asyncio.sleep(0.01)  # Small delay to prevent busy loop
                
            except can.CanError as e:
                print(f"⚠️ CAN receive error: {e}")
                await asyncio.sleep(0.1)
            except Exception as e:
                # Timeout or other errors are expected, just continue
                await asyncio.sleep(0.01)
    
    async def _can_sender(self):
        while True:
            await asyncio.sleep(1.0 / CAN_SEND_RATE_HZ)
            
            control_string = self.latest_control_string
            if not control_string or not self.can_bus:
                continue

            if LOG_INCOMING_MESSAGES:
                print(f"← RX: {control_string}")

            # **UPDATED**: Initialize the dictionary with the new l2/r2 keys.
            drive_speeds = {key: 0 for key in DRIVE_COMMAND_KEYS}
            
            commands = control_string.split(';')
            
            for command in commands:
                if '=' not in command: continue
                key, value_str = command.split('=')
                
                # Handle tuning commands
                if key in TUNING_COMMANDS:
                    try:
                        # Parse value based on command type
                        if key in ["tune_motor_dir", "tune_sensor_dir"]:
                            value = bool(int(value_str))
                        elif key in ["tune_setpoint", "tune_p"]:
                            value = float(value_str)
                        else:
                            value = int(value_str)
                        
                        send_tuning_command(self.can_bus, key, value)
                    except ValueError as e:
                        print(f"⚠️ Invalid tuning value for {key}: {value_str} ({e})")
                    continue
                
                # Handle regular commands
                try:
                    value = int(value_str)
                    if key in COMMAND_TO_CAN_ID_MAP:
                        payload = bytearray([0x01]) # Hardcoding 0x01 as this seems to be its only use now
                        payload.extend(struct.pack('>H', value))
                        can_msg = can.Message(arbitration_id=COMMAND_TO_CAN_ID_MAP[key], data=payload, is_extended_id=False)
                        try:
                            self.can_bus.send(can_msg, timeout=0.02)
                        except can.CanError as e:
                            print(f"⚠️ CAN send error (dropping msg): {e}")
                    elif key in DRIVE_COMMAND_KEYS:
                        drive_speeds[key] = value
                except ValueError:
                    continue
            
            # **UPDATED**: Assemble the payload without the leading command byte.
            # It now contains 8 bytes of data, in the order defined by DRIVE_COMMAND_KEYS.
            drive_payload = bytearray([drive_speeds[k] for k in DRIVE_COMMAND_KEYS])
            
            drive_msg = can.Message(arbitration_id=DRIVE_SYSTEM_ID, data=drive_payload, is_extended_id=False)
            try:
                self.can_bus.send(drive_msg, timeout=0.02)
            except can.CanError as e:
                print(f"⚠️ CAN send error (dropping drive msg): {e}")

    async def _create_and_send_offer(self, ws):
        if self.pc and self.pc.connectionState != "closed":
            print("Closing existing peer connection...")
            await self.pc.close()
        if self.track:
            self.track.close()

        print("Creating new peer connection...")
        cfg = RTCConfiguration([RTCIceServer(urls="stun:stun.l.google.com:19302")])
        self.pc = RTCPeerConnection(cfg)

        @self.pc.on("icecandidate")
        async def on_ice(evt):
            if evt.candidate:
                await ws.send(json.dumps({"type": "candidate", "candidate": evt.candidate.to_json()}))

        @self.pc.on("connectionstatechange")
        async def on_state():
            logging.info("Connection state: %s", self.pc.connectionState)

        self.track = TripleCameraVideoTrack(frame_rate=self.args.framerate)
        self.pc.addTrack(self.track)
        
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        
        print("🔸 Sending offer")
        await ws.send(json.dumps({"type": "offer", "sdp": self.pc.localDescription.sdp}))

    async def run(self):
        setup_can_interface()
        try:
            self.can_bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
            print("🔸 CAN bus open")
        except Exception as e:
            print(f"CAN bus error: {e}. Continuing without CAN control.")
        
        uri = "ws://64.225.55.176:8080"
        print(f"🔗 Connecting to WebSocket: {uri}")
        
        while True:
            try:
                async with websockets.connect(uri, ping_interval=30, ping_timeout=30) as ws:
                    print("🔸 WS open")
                    self.ws = ws  # Store for telemetry forwarding
                    await self._create_and_send_offer(ws)
                    reader_task = asyncio.create_task(self._handle_webrtc_messages(ws))
                    sender_task = asyncio.create_task(self._can_sender())
                    receiver_task = asyncio.create_task(self._can_receiver())
                    done, pending = await asyncio.wait([reader_task, sender_task, receiver_task], return_when=asyncio.FIRST_COMPLETED)
                    for task in pending: task.cancel()
            except (ConnectionClosedOK, ConnectionClosedError, ConnectionRefusedError) as e:
                print(f"ℹ️ WebSocket connection issue: {e}. Reconnecting in 5 seconds...")
            except Exception as e:
                print(f"⚠️ An unexpected error occurred: {e}. Reconnecting in 5 seconds...")
            finally:
                if self.pc and self.pc.connectionState != "closed": await self.pc.close()
                if self.track: self.track.close()
                await asyncio.sleep(5)

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--framerate", type=float, default=15)
    args = p.parse_args()
    
    controller = RobotController(args)
    try:
        asyncio.run(controller.run())
    except KeyboardInterrupt:
        print("\nExiting…")
    finally:
        if controller.can_bus:
            controller.can_bus.shutdown()
            os.system(f'sudo ip link set {CAN_INTERFACE} down')
        sys.exit(0)
