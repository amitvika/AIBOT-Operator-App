#!/usr/bin/env python3
"""
vidcan_merged.py (Real-Time Final Version)
────────────────────────────────────────────
• **FIX:** Re-architected control handling to eliminate command lag. A dedicated
  reader task grabs the latest WebSocket message, and a separate sender task
  sends it to the CAN bus on a fixed timer, ensuring only the most recent
  command is ever processed.
• Contains all previous stability fixes.
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
MOVE_COMMAND_BYTE = 0x01
DRIVE_SYSTEM_ID = 0x200
COMMAND_TO_CAN_ID_MAP = {
    "h1X": 0x101, "h1Y": 0x102, "h1Z": 0x103, "h1p": 0x104, "h1y": 0x105, "h1r": 0x106, "g1":  0x110,
    "h2X": 0x121, "h2Y": 0x122, "h2Z": 0x123, "h2p": 0x124, "h2y": 0x125, "h2r": 0x126, "g2":  0x130,
}
DRIVE_COMMAND_KEYS = ['f', 'b', 'l', 'r', 'u', 'd']
CAN_SEND_RATE_HZ = 20 # Rate at which we send CAN messages

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

    async def _handle_webrtc_messages(self, ws):
        """Handles ICE, Answer, and command messages from the server."""
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
                # This is the "Reader" part: just update the latest string.
                self.latest_control_string = data.get("controlString")
            elif msg_type == "new_offer_request":
                print("🔄 Re-offer request received...")
                await self._create_and_send_offer(ws)

    async def _can_sender(self):
        """
        This is the "Sender" task. It runs on a fixed timer, grabs the latest
        command, and sends it to the CAN bus.
        """
        while True:
            await asyncio.sleep(1.0 / CAN_SEND_RATE_HZ)
            
            control_string = self.latest_control_string
            if not control_string or not self.can_bus:
                continue

            if LOG_INCOMING_MESSAGES:
                print(f"← RX: {control_string}")

            drive_speeds = {'f': 0, 'b': 0, 'l': 0, 'r': 0, 'u': 0, 'd': 0}
            commands = control_string.split(';')
            
            for command in commands:
                if '=' not in command: continue
                key, value_str = command.split('=')
                try:
                    value = int(value_str)
                    if key in COMMAND_TO_CAN_ID_MAP:
                        payload = bytearray([MOVE_COMMAND_BYTE])
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
            
            drive_payload = bytearray([MOVE_COMMAND_BYTE] + [drive_speeds[k] for k in DRIVE_COMMAND_KEYS])
            drive_msg = can.Message(arbitration_id=DRIVE_SYSTEM_ID, data=drive_payload, is_extended_id=False)
            try:
                self.can_bus.send(drive_msg, timeout=0.02)
            except can.CanError as e:
                print(f"⚠️ CAN send error (dropping drive msg): {e}")

    async def _create_and_send_offer(self, ws):
        """Creates a new WebRTC peer connection, track, and sends the offer."""
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
        """Main execution function."""
        setup_can_interface()
        try:
            self.can_bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
            print("🔸 CAN bus open")
        except Exception as e:
            print(f"CAN bus error: {e}. Continuing without CAN control.")
        
        uri = "ws://64.225.55.176:8080"
        
        while True:
            try:
                async with websockets.connect(uri, ping_interval=30, ping_timeout=30) as ws:
                    print("🔸 WS open")
                    await self._create_and_send_offer(ws)

                    # Create and run the two main tasks in parallel
                    reader_task = asyncio.create_task(self._handle_webrtc_messages(ws))
                    sender_task = asyncio.create_task(self._can_sender())
                    
                    # Wait for either task to complete (e.g., if the connection drops)
                    done, pending = await asyncio.wait(
                        [reader_task, sender_task],
                        return_when=asyncio.FIRST_COMPLETED,
                    )

                    # Clean up pending tasks before reconnecting
                    for task in pending:
                        task.cancel()
                    
            except (ConnectionClosedOK, ConnectionClosedError, ConnectionRefusedError) as e:
                print(f"ℹ️ WebSocket connection issue: {e}. Reconnecting in 5 seconds...")
            except Exception as e:
                print(f"⚠️ An unexpected error occurred: {e}. Reconnecting in 5 seconds...")
            finally:
                if self.pc and self.pc.connectionState != "closed":
                    await self.pc.close()
                if self.track:
                    self.track.close()
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