import math
import numpy as np
import pybullet as p
import pybullet_data
from numba import jit
import os
import time
import threading
import base64
import json
import serial
import cv2
import cv2.aruco as aruco
import webview
from collections import deque
import queue

# --- CHANGE 1: Added a threading event for graceful shutdown ---
shutdown_event = threading.Event()

# --- Core Robot Logic & Kinematics ---
PI = math.pi
TWO_PI = 2.0 * PI
RAD_TO_12BIT_SCALE = 4095.0 / TWO_PI
joint_zero_offsets = np.array([PI/2, PI/2, 0, PI, 0, -0.5], dtype=np.float32)

# --- PyBullet Setup ---
try:
    physicsClientId = p.connect(p.DIRECT)
except p.error:
    physicsClientId = p.connect(p.CPU)
    
p.setAdditionalSearchPath(pybullet_data.getDataPath())

try:
    orientation_robot1 = p.getQuaternionFromEuler([0, -math.pi/2, 0])
    robot_id1 = p.loadURDF("ur10.urdf", basePosition=[-0.175, 0, 0], baseOrientation=orientation_robot1, useFixedBase=True, physicsClientId=physicsClientId)
    orientation_robot2 = p.getQuaternionFromEuler([0, math.pi/2, 0])
    robot_id2 = p.loadURDF("ur10.urdf", basePosition=[0.175, 0, 0], baseOrientation=orientation_robot2, useFixedBase=True, physicsClientId=physicsClientId)
except p.error as e:
    print("CRITICAL ERROR: Could not load 'ur10.urdf'.")
    raise e

def get_robot_kinematic_info(robot_id):
    num_joints = p.getNumJoints(robot_id, physicsClientId=physicsClientId)
    movable_joints = [i for i in range(num_joints) if p.getJointInfo(robot_id, i, physicsClientId=physicsClientId)[2] != p.JOINT_FIXED]
    end_effector_link_index = movable_joints[-1] if movable_joints else -1
    lower_limits, upper_limits, joint_ranges, rest_poses, joint_names = [], [], [], [], []
    
    for i in movable_joints:
        info = p.getJointInfo(robot_id, i, physicsClientId=physicsClientId)
        joint_name = info[1].decode('utf-8')
        lower_limit = info[8]
        upper_limit = info[9]
        
        # Handle unlimited joints (PyBullet returns -1 for unlimited)
        if lower_limit == -1:
            lower_limit = -math.pi
        if upper_limit == -1:
            upper_limit = math.pi
            
        lower_limits.append(lower_limit)
        upper_limits.append(upper_limit)
        joint_ranges.append(upper_limit - lower_limit)
        rest_poses.append(0)  # Default rest pose at middle of range
        joint_names.append(joint_name)
        
        print(f"Joint {i} ({joint_name}): limits [{lower_limit:.3f}, {upper_limit:.3f}] rad")
    
    return {
        "movable_joints": movable_joints, 
        "end_effector_link": end_effector_link_index, 
        "lower_limits": lower_limits, 
        "upper_limits": upper_limits, 
        "joint_ranges": joint_ranges, 
        "rest_poses": rest_poses,
        "joint_names": joint_names
    }

robot_info1 = get_robot_kinematic_info(robot_id1)
robot_info2 = get_robot_kinematic_info(robot_id2)

# --- Helper Functions & Global State ---
def rad_to_12bit(angle_rad):
    return max(0, min(4095, int(round((angle_rad % TWO_PI) * RAD_TO_12BIT_SCALE))))

current_q1 = np.zeros(len(robot_info1["movable_joints"]), dtype=np.float32)
current_q2 = np.zeros(len(robot_info2["movable_joints"]), dtype=np.float32)
webview_window, latest_serial_data = None, "controller not connected"
robot1_zeroing, robot2_zeroing = False, False
last_detection_robot1, last_detection_robot2 = 0, 0
latest_ui_frame, latest_ui_frame_lock = None, threading.Lock()
FIXED_POS_SCALE, FIXED_ANGLE_SCALE = np.array([-2.625,-1.125,-2.25]), np.array([-1.,-1.,-1.])  # 15x larger for better movement
OFFSET_ROBOT1_POS, OFFSET_ROBOT1_ORI = np.array([0.,1.5,0.3]), np.array([-90.,0.,0.])  # 15x larger for better movement
OFFSET_ROBOT2_POS, OFFSET_ROBOT2_ORI = np.array([0.,1.5,0.3]), np.array([-90.,0.,0.])  # 15x larger for better movement
ARUCO_MAPPING = {'x':0,'y':2,'z':1,'roll':0,'pitch':2,'yaw':1}
filter_history = {'1':{k:deque(maxlen=3) for k in ARUCO_MAPPING}, '2':{k:deque(maxlen=3) for k in ARUCO_MAPPING}}

def is_at_zero(q, tol=0.01): return all(abs(c) <= tol for c in q)

def validate_joint_limits(joint_angles, robot_info):
    """Validate joint angles against limits and return clamped values and violations"""
    clamped_angles = np.array(joint_angles, dtype=np.float32)
    violations = []
    
    for i, (angle, lower, upper) in enumerate(zip(joint_angles, robot_info["lower_limits"], robot_info["upper_limits"])):
        if angle < lower:
            clamped_angles[i] = lower
            violations.append(f"Joint {i} ({robot_info['joint_names'][i]}): {angle:.3f} < {lower:.3f}")
        elif angle > upper:
            clamped_angles[i] = upper
            violations.append(f"Joint {i} ({robot_info['joint_names'][i]}): {angle:.3f} > {upper:.3f}")
    
    return clamped_angles, violations

def check_joint_limit_violations(joint_angles, robot_info):
    """Check if joint angles violate limits without clamping"""
    violations = []
    for i, (angle, lower, upper) in enumerate(zip(joint_angles, robot_info["lower_limits"], robot_info["upper_limits"])):
        if angle < lower or angle > upper:
            violations.append({
                'joint_index': i,
                'joint_name': robot_info['joint_names'][i],
                'current_angle': angle,
                'lower_limit': lower,
                'upper_limit': upper,
                'violation_amount': min(angle - lower, upper - angle) if angle < lower else angle - upper
            })
    return violations

# --- Kinematics Functions ---
def pybullet_inverse_kinematics(robot_id_num, robot_info, x, y, z, roll, pitch, yaw):
    target_pos = [x, y, z]
    target_ori = p.getQuaternionFromEuler([math.radians(roll), math.radians(pitch), math.radians(yaw)])
    
    # Use middle of joint ranges as rest poses for better IK convergence
    rest_poses = [(lower + upper) / 2 for lower, upper in zip(robot_info["lower_limits"], robot_info["upper_limits"])]
    
    joint_poses = p.calculateInverseKinematics(
        robot_id_num, 
        robot_info["end_effector_link"], 
        target_pos, 
        target_ori, 
        lowerLimits=robot_info["lower_limits"], 
        upperLimits=robot_info["upper_limits"], 
        jointRanges=robot_info["joint_ranges"], 
        restPoses=rest_poses, 
        solver=0, 
        maxNumIterations=100,  # Increased for better convergence
        residualThreshold=.001,  # Tighter tolerance
        physicsClientId=physicsClientId
    )
    
    result_angles = np.array(joint_poses[:len(robot_info["movable_joints"])], dtype=np.float32)
    
    # Validate and clamp results to joint limits
    clamped_angles, violations = validate_joint_limits(result_angles, robot_info)
    
    if violations:
        print(f"Joint limit violations in IK solution: {violations}")
    
    return clamped_angles

def pybullet_forward_kinematics(robot_id_num, robot_info, joint_angles):
    # Validate joint angles before setting them
    violations = check_joint_limit_violations(joint_angles, robot_info)
    if violations:
        print(f"Joint limit violations in FK input: {[v['joint_name'] + f' ({v['current_angle']:.3f})' for v in violations]}")
    
    for i, joint_index in enumerate(robot_info["movable_joints"]):
        p.resetJointState(robot_id_num, joint_index, joint_angles[i], physicsClientId=physicsClientId)
    link_positions = []
    base_pos, _ = p.getBasePositionAndOrientation(robot_id_num, physicsClientId=physicsClientId)
    link_positions.append(list(base_pos))
    for i in robot_info["movable_joints"]:
        link_state = p.getLinkState(robot_id_num, i, computeForwardKinematics=True, physicsClientId=physicsClientId)
        link_positions.append(list(link_state[0]))
    
    # Add end effector (tool0) position and orientation
    num_joints = p.getNumJoints(robot_id_num, physicsClientId=physicsClientId)
    # Find the tool0 link
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id_num, i, physicsClientId=physicsClientId)
        if joint_info[12].decode('utf-8') == 'tool0':  # joint_info[12] is the child link name
            tool0_state = p.getLinkState(robot_id_num, i, computeForwardKinematics=True, physicsClientId=physicsClientId)
            link_positions.append(list(tool0_state[0]))  # Position
            link_positions.append(list(tool0_state[1]))  # Orientation quaternion [x, y, z, w]
            break
    else:
        # Fallback: if tool0 not found, use the last movable joint position and identity orientation
        if link_positions:
            link_positions.append(link_positions[-1])  # Position
            link_positions.append([0, 0, 0, 1])  # Identity quaternion
    
    return link_positions

# --- Threaded Functions ---
def move_all_joints_to_zero(robot_id):
    global robot1_zeroing, robot2_zeroing
    q = current_q1 if robot_id == 1 else current_q2
    target_q, start_time, duration, initial_q = np.zeros_like(q), time.time(), 2.0, np.copy(q)
    while time.time() - start_time < duration:
        if shutdown_event.is_set(): break
        progress = (time.time() - start_time) / duration
        q[:] = initial_q + (target_q - initial_q) * progress
        time.sleep(0.02)
    q[:] = target_q
    if robot_id == 1: robot1_zeroing = False
    else: robot2_zeroing = False

def transform_aruco_marker(marker_data, robot_id):
    pos, ori = np.array(marker_data['position']), np.array(marker_data['orientation'])
    ori[0] += 180 if ori[0] < 0 else -180
    remap_idx = [ARUCO_MAPPING[k] for k in ['x','y','z','roll','pitch','yaw']]
    remapped = np.concatenate((pos[remap_idx[:3]], ori[remap_idx[3:]]))
    offset_pos, offset_ori = (OFFSET_ROBOT1_POS, OFFSET_ROBOT1_ORI) if robot_id == 1 else (OFFSET_ROBOT2_POS, OFFSET_ROBOT2_ORI)
    offsets, scales = np.concatenate((offset_pos, offset_ori)), np.concatenate((FIXED_POS_SCALE, FIXED_ANGLE_SCALE))
    scaled = remapped * scales + offsets
    raw = dict(zip(['x', 'y', 'z', 'roll', 'pitch', 'yaw'], scaled))
    history = filter_history[str(robot_id)]
    return {k: sum(d)/(len(d) or 1) for k,v in raw.items() if (d:=history[k]).append(v) or True}

def read_serial_data():
    global latest_serial_data
    # --- CHANGE 2: All while loops now check the shutdown_event ---
    while not shutdown_event.is_set():
        try:
            with serial.Serial("COM4", baudrate=9600, timeout=1) as ser:
                while not shutdown_event.is_set():
                    if ser.in_waiting > 0: latest_serial_data = ser.readline().decode('utf-8', 'replace').strip()
                    else: time.sleep(0.1)
        except serial.SerialException:
            latest_serial_data = "controller not connected"
            time.sleep(2)

def camera_capture_thread(cap, frame_queue):
    while not shutdown_event.is_set():
        ret, frame = cap.read()
        if ret:
            if frame_queue.full():
                try: frame_queue.get_nowait()
                except queue.Empty: pass
            frame_queue.put(frame)
        else: time.sleep(0.01)

def rotationMatrixToEulerAngles(R):
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    return np.degrees([np.arctan2(R[2,1],R[2,2]),np.arctan2(-R[2,0],sy),np.arctan2(R[1,0],R[0,0])]) if sy > 1e-6 else np.degrees([np.arctan2(-R[1,2],R[1,1]),np.arctan2(-R[2,0],sy),0])

def update_ui_feed():
    with latest_ui_frame_lock: frame_to_render = latest_ui_frame
    if frame_to_render is not None and webview_window:
        ret, buf = cv2.imencode('.jpg', frame_to_render, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ret:
            js_code = f"document.getElementById('aruco_feed').src='data:image/jpeg;base64,{base64.b64encode(buf).decode('utf-8')}';"
            try: webview_window.evaluate_js(js_code)
            except Exception: pass
    if not shutdown_event.is_set():
        threading.Timer(1.0 / 30.0, update_ui_feed).start()

def run_aruco(frame_queue, api_instance):
    global last_detection_robot1, last_detection_robot2, robot1_zeroing, robot2_zeroing, latest_ui_frame, latest_ui_frame_lock
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    cam_matrix, dist_coeffs = np.array([[800,0,320],[0,800,240],[0,0,1]],dtype=np.float64), np.zeros(4)
    while not shutdown_event.is_set():
        try:
            frame = frame_queue.get(timeout=0.1)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.06, cam_matrix, dist_coeffs)
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id not in [0, 1]: continue
                    robot_id = marker_id + 1
                    if robot_id == 1: last_detection_robot1 = time.time()
                    else: last_detection_robot2 = time.time()
                    R, _ = cv2.Rodrigues(rvecs[i])
                    marker_data = {"position": tvecs[i][0], "orientation": rotationMatrixToEulerAngles(R)}
                    processed_target = transform_aruco_marker(marker_data, robot_id)
                    ik_result = api_instance.update_target(robot_id, **processed_target)
                    ik_result['debug_pose'] = processed_target
                    if webview_window:
                        js_code = f"handle_robot_update({robot_id}, {json.dumps(ik_result)});"
                        try: webview_window.evaluate_js(js_code)
                        except Exception: pass
            with latest_ui_frame_lock: latest_ui_frame = frame
            now = time.time()
            if (now - last_detection_robot1 > 3) and not is_at_zero(current_q1) and not robot1_zeroing:
                robot1_zeroing = True
                threading.Thread(target=move_all_joints_to_zero, args=(1,), daemon=True).start()
            if (now - last_detection_robot2 > 3) and not is_at_zero(current_q2) and not robot2_zeroing:
                robot2_zeroing = True
                threading.Thread(target=move_all_joints_to_zero, args=(2,), daemon=True).start()
        except queue.Empty: continue
        except Exception as e: print(f"Error in aruco loop: {e}")

# --- API Class & Main Execution ---
class API:
    def get_latest_serial(self): return {"serial": latest_serial_data}
    def update_target(self, rid, x, y, z, roll, pitch, yaw):
        q, rob_id, rob_info, last, zeroing = (current_q1, robot_id1, robot_info1, last_detection_robot1, robot1_zeroing) if rid == 1 else (current_q2, robot_id2, robot_info2, last_detection_robot2, robot2_zeroing)
        if zeroing or (time.time() - last > 3):
            pos = pybullet_forward_kinematics(rob_id, rob_info, q)
            violations = check_joint_limit_violations(q, rob_info)
            return {
                "positions": pos, 
                "joint_angles": [rad_to_12bit(a + o) for a, o in zip(q, joint_zero_offsets)],
                "joint_limit_violations": violations,
                "joint_limits": {
                    "lower": rob_info["lower_limits"],
                    "upper": rob_info["upper_limits"],
                    "names": rob_info["joint_names"]
                }
            }
        new_q = pybullet_inverse_kinematics(rob_id, rob_info, x, y, z, roll, pitch, yaw)
        if new_q is not None and len(new_q) == len(q): q[:] = new_q
        pos = pybullet_forward_kinematics(rob_id, rob_info, q)
        violations = check_joint_limit_violations(q, rob_info)
        return {
            "positions": pos, 
            "joint_angles": [rad_to_12bit(a + o) for a, o in zip(q, joint_zero_offsets)],
            "joint_limit_violations": violations,
            "joint_limits": {
                "lower": rob_info["lower_limits"],
                "upper": rob_info["upper_limits"],
                "names": rob_info["joint_names"]
            }
        }
    def go_to_zero_robot(self, rid):
        q, rob_id, rob_info = (current_q1, robot_id1, robot_info1) if rid == 1 else (current_q2, robot_id2, robot_info2)
        if rid == 1: global robot1_zeroing; robot1_zeroing = True
        else: global robot2_zeroing; robot2_zeroing = True
        threading.Thread(target=move_all_joints_to_zero, args=(rid,), daemon=True).start()
        pos = pybullet_forward_kinematics(rob_id, rob_info, np.zeros_like(q))
        violations = check_joint_limit_violations(np.zeros_like(q), rob_info)
        return {
            f"positions{rid}": pos, 
            f"joint_angles{rid}": [rad_to_12bit(a + o) for a,o in zip(np.zeros_like(q), joint_zero_offsets)],
            f"joint_limit_violations{rid}": violations,
            f"joint_limits{rid}": {
                "lower": rob_info["lower_limits"],
                "upper": rob_info["upper_limits"],
                "names": rob_info["joint_names"]
            }
        }
    def go_to_zero_robot1(self): return self.go_to_zero_robot(1)
    def go_to_zero_robot2(self): return self.go_to_zero_robot(2)

# --- CHANGE 3: This function will be called by pywebview when the window is closed ---
def on_window_closed():
    print("Webview window closed by user.")
    shutdown_event.set()

if __name__ == '__main__':
    api_main = API()
    cap = cv2.VideoCapture(0)
    try:
        last_detection_robot1, last_detection_robot2 = time.time(), time.time()
        threading.Thread(target=read_serial_data, daemon=True).start()
        if not cap.isOpened(): print("Error: Could not open video stream.")
        else:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            frame_queue = queue.Queue(maxsize=1)
            threading.Thread(target=camera_capture_thread, args=(cap,frame_queue), daemon=True).start()
            threading.Thread(target=run_aruco, args=(frame_queue, api_main), daemon=True).start()
            threading.Timer(1.0, update_ui_feed).start()
        
        # --- CHANGE 4: Pass the on_closed handler to the window ---
        webview_window = webview.create_window("Robot Control", "index.html", js_api=api_main, width=1600, height=900, resizable=True)
        webview_window.events.closed += on_window_closed
        webview.start(debug=True)
    
    except KeyboardInterrupt:
        print("Ctrl+C detected.")
        shutdown_event.set()

    finally:
        # --- CHANGE 5: Set event one last time and wait briefly for threads to exit ---
        print("\nShutting down...")
        if not shutdown_event.is_set():
            shutdown_event.set()
        time.sleep(0.2) # Give threads a moment to check the event and exit
        if cap.isOpened():
            cap.release()
        p.disconnect(physicsClientId=physicsClientId)
        print("Cleanup complete. Goodbye!")