from flask import Flask, Response, jsonify, request, render_template
import cv2
import numpy as np
from ultralytics import YOLO
import requests
import threading
import time
from collections import deque
import cv2.aruco as aruco 
import os

app = Flask(__name__)

ESP32_IP = None
ESP8266_IP = None

MODEL_PATH = "my_model_416n.pt"
YOLO_HIGH_CONFIDENCE = 0.80
YOLO_IMG_SIZE = 416

MARKER_DIST_03 = 22.9
MARKER_DIST_12 = 22.9
MARKER_DIST_01 = 13.2
MARKER_DIST_32 = 13.1

BOARD_Z_OFFSET = 0.3
ROBOT_OFFSET_X = -11.70
ROBOT_OFFSET_Y = 10.15

CAMERA_MATRIX_PATH = "camera_matrix.npy"
camera_matrix = None
dist_coeffs = None
camera_height_cm = None

ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
ARUCO_PARAMS = aruco.DetectorParameters()
ARUCO_DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

ID_BL = 0
ID_TL = 1
ID_TR = 2
ID_BR = 3

homography_matrix = None
board_detected = False
BOARD_WIDTH_CM = None
BOARD_HEIGHT_CM = None
camera_position_board = None

ADAPTIVE_THRESH_BLOCK_SIZE = 201
ADAPTIVE_THRESH_C = -10
MIN_CONTOUR_AREA = 800
MAX_CONTOUR_AREA = 50000

MORPH_KERNEL_OPEN = np.ones((1, 1), np.uint8)
MORPH_ITERATIONS_OPEN = 2
MORPH_KERNEL_CLOSE = np.ones((5, 5), np.uint8)
MORPH_ITERATIONS_CLOSE = 3

ROBOT_BASE_X = 0.0
ROBOT_BASE_Y = -12.0

BOARD_MASK_PATH = "board_mask.png"
BOARD_MASK = None
BOARD_MASK_RESIZED = None

OBJECT_STABILITY_TIME = 1.0
POSITION_CHANGE_THRESHOLD = 2.0
SKIP_FRAMES = 0

model = YOLO(MODEL_PATH, task='detect')
model.fuse()
labels = model.names

streaming = False
current_frame = None
frame_lock = threading.Lock()
frame_queue = deque(maxlen=2)
fps_counter = {'count': 0, 'start': time.time(), 'fps': 0}

detected_objects = {}
sent_objects = set()
current_pick_in_progress = False
last_detection_time = 0

def load_camera_parameters():
    global camera_matrix, dist_coeffs
    
    try:
        if not os.path.exists("camera_matrix.npy"):
            return False
            
        camera_matrix = np.load("camera_matrix.npy")
        
        try:
            dist_coeffs = np.load("dist_coeffs.npy")
        except:
            dist_coeffs = None
        
        return True
        
    except Exception as e:
        return False

def get_object_surface_height(classname):
    object_heights = {
        'Cuboid': 1.35,
        'Square': 1.35,
        'Unknown': 1.35
    }
    return object_heights.get(classname, 1.35)

def update_board_pose(frame):
    global homography_matrix, board_detected, BOARD_WIDTH_CM, BOARD_HEIGHT_CM
    global camera_height_cm, camera_position_board
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = ARUCO_DETECTOR.detectMarkers(gray)
    
    if ids is None or len(ids) < 4:
        board_detected = False
        return
        
    corner_dict = {}
    for i, marker_id in enumerate(ids.flatten()):
        c = corners[i][0]
        cx = float(np.mean(c[:, 0]))
        cy = float(np.mean(c[:, 1]))
        corner_dict[marker_id] = (cx, cy)
    
    if not all(k in corner_dict for k in [ID_BL, ID_TL, ID_TR, ID_BR]):
        board_detected = False
        return
    
    BOARD_WIDTH_CM = (MARKER_DIST_03 + MARKER_DIST_12) / 2
    BOARD_HEIGHT_CM = (MARKER_DIST_01 + MARKER_DIST_32) / 2
    
    src_pts = np.array([
        corner_dict[ID_BL],
        corner_dict[ID_BR],
        corner_dict[ID_TR],
        corner_dict[ID_TL]
    ], dtype=np.float32)
    
    dst_pts = np.array([
        [0, 0],
        [BOARD_WIDTH_CM, 0],
        [BOARD_WIDTH_CM, BOARD_HEIGHT_CM],
        [0, BOARD_HEIGHT_CM]
    ], dtype=np.float32)
    
    H, _ = cv2.findHomography(src_pts, dst_pts)
    homography_matrix = H
    board_detected = True
    
    if camera_matrix is not None and homography_matrix is not None:
        cx_px = camera_matrix[0, 2]
        cy_px = camera_matrix[1, 2]
        
        point = np.array([[[cx_px, cy_px]]], dtype=np.float32)
        nadir_board = cv2.perspectiveTransform(point, homography_matrix)
        camera_position_board = (float(nadir_board[0][0][0]), float(nadir_board[0][0][1]))
        
        object_points_3d = np.array([
            [0, 0, 0],
            [BOARD_WIDTH_CM, 0, 0],
            [BOARD_WIDTH_CM, BOARD_HEIGHT_CM, 0],
            [0, BOARD_HEIGHT_CM, 0]
        ], dtype=np.float32)
        
        image_points_2d = np.array([
            corner_dict[ID_BL],
            corner_dict[ID_BR],
            corner_dict[ID_TR],
            corner_dict[ID_TL]
        ], dtype=np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            object_points_3d,
            image_points_2d,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if success:
            camera_height_cm = abs(tvec[2][0])

def get_board_coords(u, v, object_height_cm=0.0):
    global homography_matrix, camera_height_cm, camera_position_board
    
    if homography_matrix is None:
        return None, None

    point = np.array([[[u, v]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(point, homography_matrix)
    
    x_board_raw = dst[0][0][0]
    y_board_raw = dst[0][0][1]
    
    if object_height_cm == 0.0 or camera_height_cm is None or camera_position_board is None:
        return x_board_raw, y_board_raw
    
    h = BOARD_Z_OFFSET + object_height_cm
    H = camera_height_cm - BOARD_Z_OFFSET
    camera_x, camera_y = camera_position_board
    
    correction_factor = h / H
    
    x_board_corrected = x_board_raw - (x_board_raw - camera_x) * correction_factor
    y_board_corrected = y_board_raw - (y_board_raw - camera_y) * correction_factor
    
    return x_board_corrected, y_board_corrected

def board_to_robot(x_board, y_board):
    x_robot = x_board + ROBOT_OFFSET_X
    y_robot = y_board + ROBOT_OFFSET_Y
    return x_robot, y_robot

def load_board_mask():
    global BOARD_MASK
    try:
        mask = cv2.imread(BOARD_MASK_PATH, cv2.IMREAD_GRAYSCALE)
        if mask is not None:
            BOARD_MASK = mask
            return True
        return False
    except Exception as e:
        return False

def adaptive_threshold_segmentation(frame):
    global BOARD_MASK_RESIZED
    try:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v_channel = hsv[:, :, 2]
        binary_img = cv2.adaptiveThreshold(v_channel, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                             cv2.THRESH_BINARY, ADAPTIVE_THRESH_BLOCK_SIZE, ADAPTIVE_THRESH_C)
        
        cleaned_binary_img = cv2.morphologyEx(binary_img, cv2.MORPH_OPEN, MORPH_KERNEL_OPEN, iterations=MORPH_ITERATIONS_OPEN)
        cleaned_binary_img = cv2.morphologyEx(cleaned_binary_img, cv2.MORPH_CLOSE, MORPH_KERNEL_CLOSE, iterations=MORPH_ITERATIONS_CLOSE)
        
        if BOARD_MASK is not None:
            if BOARD_MASK_RESIZED is None or BOARD_MASK_RESIZED.shape != cleaned_binary_img.shape:
                BOARD_MASK_RESIZED = cv2.resize(BOARD_MASK, (cleaned_binary_img.shape[1], cleaned_binary_img.shape[0]), 
                                                interpolation=cv2.INTER_NEAREST)
            cleaned_binary_img = cv2.bitwise_and(cleaned_binary_img, cleaned_binary_img, mask=BOARD_MASK_RESIZED)
        
        return cleaned_binary_img
    except Exception as e:
        return None

def detect_unknown_objects_adaptive(frame):
    binary_mask = adaptive_threshold_segmentation(frame)
    if binary_mask is None:
        return []
    
    try:
        contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_objects = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx = x + w // 2
                cy = y + h // 2
            
            detected_objects.append({
                'bbox': (x, y, x + w, y + h),
                'center': (cx, cy),
                'area': area
            })
        
        return detected_objects
    except Exception as e:
        return []

def iou_bbox(box1, box2):
    x1, y1, x2, y2 = box1
    x1_, y1_, x2_, y2_ = box2
    xi1, yi1 = max(x1, x1_), max(y1, y1_)
    xi2, yi2 = min(x2, x2_), min(y2, y2_)
    inter_w = max(0, xi2 - xi1)
    inter_h = max(0, yi2 - yi1)
    if inter_w == 0 or inter_h == 0:
        return 0
    inter_area = inter_w * inter_h
    box1_area = (x2 - x1) * (y2 - y1)
    box2_area = (x2_ - x1_) * (y2_ - y1_)
    union_area = box1_area + box2_area - inter_area
    return inter_area / union_area if union_area > 0 else 0

def filter_unknown_objects(yolo_detections, contour_detections):
    if not yolo_detections:
        return contour_detections
    
    MIN_IOU = 0.30
    MIN_CENTER_DIST = 50
    
    unknown_objects = []
    yolo_data = [((d['bbox'][0] + d['bbox'][2]) / 2, (d['bbox'][1] + d['bbox'][3]) / 2, d['bbox']) 
                 for d in yolo_detections]
    
    for cont_det in contour_detections:
        cont_bbox = cont_det['bbox']
        cont_cx, cont_cy = cont_det['center']
        is_unknown = True
        
        for yolo_cx, yolo_cy, yolo_bbox in yolo_data:
            center_dist_sq = (cont_cx - yolo_cx)**2 + (cont_cy - yolo_cy)**2
            if center_dist_sq < MIN_CENTER_DIST**2:
                is_unknown = False
                break
            if iou_bbox(yolo_bbox, cont_bbox) >= MIN_IOU:
                is_unknown = False
                break
        
        if is_unknown:
            unknown_objects.append(cont_det)
    
    return unknown_objects

def calculate_distance_from_base(x, y):
    dx = x - ROBOT_BASE_X
    dy = y - ROBOT_BASE_Y
    return (dx**2 + dy**2) ** 0.5

def get_object_id(name, x, y):
    return f"{name}_{int(x*10)}_{int(y*10)}"

def send_object_to_esp(obj_id, obj_data):
    global current_pick_in_progress, sent_objects
    
    if ESP8266_IP is None:
        return False
    
    if current_pick_in_progress:
        return False
    
    try:
        payload = {
            "name": obj_data['name'],
            "x": round(obj_data['x'] * 10, 1),
            "y": round(obj_data['y'] * 10, 1),
            "z": round(obj_data['z'] * 10, 1)
        }
        
        response = requests.post(f"http://{ESP8266_IP}/add_object", json=payload, timeout=3)
        
        if response.status_code == 200:
            sent_objects.add(obj_id)
            current_pick_in_progress = True
            return True
        else:
            return False
            
    except Exception as e:
        return False

def check_and_send_stable_objects():
    if current_pick_in_progress:
        return
    
    now = time.time()
    stable_objects = []
    
    for obj_id, obj_data in detected_objects.items():
        if obj_id in sent_objects:
            continue
        
        stable_time = now - obj_data['stable_since']
        if stable_time >= OBJECT_STABILITY_TIME:
            stable_objects.append((obj_id, obj_data))
    
    if not stable_objects:
        return
    
    stable_objects.sort(key=lambda x: x[1]['distance'])
    closest_id, closest_data = stable_objects[0]
    
    send_object_to_esp(closest_id, closest_data)

def update_fps():
    fps_counter['count'] += 1
    elapsed = time.time() - fps_counter['start']
    if elapsed > 1.0:
        fps_counter['fps'] = fps_counter['count'] / elapsed
        fps_counter['count'] = 0
        fps_counter['start'] = time.time()

def capture_thread():
    while streaming:
        if ESP32_IP is None:
            time.sleep(0.5)
            continue
        
        try:
            stream = requests.get(f"http://{ESP32_IP}/stream", stream=True, timeout=5)
            bytes_buffer = bytes()
            
            for chunk in stream.iter_content(chunk_size=4096):
                if not streaming:
                    break
                
                bytes_buffer += chunk
                jpeg_start = bytes_buffer.find(b'\xff\xd8')
                jpeg_end = bytes_buffer.find(b'\xff\xd9')
                
                if jpeg_start != -1 and jpeg_end != -1:
                    jpeg_data = bytes_buffer[jpeg_start:jpeg_end + 2]
                    bytes_buffer = bytes_buffer[jpeg_end + 2:]
                    
                    frame = cv2.imdecode(np.frombuffer(jpeg_data, np.uint8), cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        frame_queue.append(frame)
                    
                    if len(bytes_buffer) > 100000:
                        bytes_buffer = bytes()
                        
        except Exception as e:
            time.sleep(2)

def detection_thread():
    global detected_objects, last_detection_time, BOARD_MASK_RESIZED
    
    frame_count = 0
    
    while streaming:
        if len(frame_queue) == 0:
            time.sleep(0.01)
            continue
        
        frame = frame_queue[-1].copy()
        frame_count += 1
        
        if SKIP_FRAMES > 0 and frame_count % (SKIP_FRAMES + 1) != 0:
            time.sleep(0.01)
            continue
        
        try:
            now = time.time()
            
            if BOARD_MASK is not None:
                 if BOARD_MASK_RESIZED is None or BOARD_MASK_RESIZED.shape != frame.shape[:2]:
                     BOARD_MASK_RESIZED = cv2.resize(BOARD_MASK, (frame.shape[1], frame.shape[0]), 
                                                     interpolation=cv2.INTER_NEAREST)

            update_board_pose(frame)
            
            if not board_detected:
                time.sleep(0.1)
                continue

            results = model(frame, imgsz=YOLO_IMG_SIZE, verbose=False, half=False)
            detections = results[0].boxes
            
            current_frame_objects = {}
            yolo_for_filter = []
            
            if detections is not None and len(detections) > 0:
                boxes_xyxy = detections.xyxy.cpu().numpy()
                confidences = detections.conf.cpu().numpy()
                class_ids = detections.cls.cpu().numpy().astype(int)
                
                for i in range(len(detections)):
                    conf = confidences[i]
                    
                    if conf < YOLO_HIGH_CONFIDENCE:
                        continue
                    
                    xmin, ymin, xmax, ymax = boxes_xyxy[i].astype(int)
                    classname = labels[class_ids[i]]
                    
                    cx = (xmin + xmax) / 2
                    cy = (ymin + ymax) / 2 
                    
                    if BOARD_MASK_RESIZED is not None:
                        try:
                            if BOARD_MASK_RESIZED[int(cy), int(cx)] == 0:
                                continue
                        except IndexError:
                            continue
                    
                    obj_height = get_object_surface_height(classname)
                    wx_board, wy_board = get_board_coords(cx, cy, obj_height)
                    
                    if wx_board is None:
                        continue
                    
                    wz_board = BOARD_Z_OFFSET + obj_height
                    wx_robot, wy_robot = board_to_robot(wx_board, wy_board)
                    
                    distance = calculate_distance_from_base(wx_robot, wy_robot)
                    obj_id = get_object_id(classname, wx_robot, wy_robot)
                    
                    obj_data = {
                        'name': classname,
                        'x': wx_robot,
                        'y': wy_robot,
                        'z': wz_board,
                        'distance': distance,
                        'bbox': (xmin, ymin, xmax, ymax),
                        'center': (int(cx), int(cy)),
                        'confidence': float(conf),
                        'type': 'yolo'
                    }
                    
                    current_frame_objects[obj_id] = obj_data
                    yolo_for_filter.append(obj_data)
            
            contour_detections = detect_unknown_objects_adaptive(frame)
            
            if len(contour_detections) > 0:
                unknown_objects = filter_unknown_objects(yolo_for_filter, contour_detections)
                
                for unknown in unknown_objects:
                    cx, cy = unknown['center']
                    xmin, ymin, xmax, ymax = unknown['bbox']
                    
                    obj_height = get_object_surface_height('Unknown')
                    wx_board, wy_board = get_board_coords(cx, cy, obj_height)
                    if wx_board is None:
                        continue
                    
                    wz_board = BOARD_Z_OFFSET + obj_height
                    wx_robot, wy_robot = board_to_robot(wx_board, wy_board)
                    
                    distance = calculate_distance_from_base(wx_robot, wy_robot)
                    obj_id = get_object_id('Unknown', wx_robot, wy_robot)
                    
                    current_frame_objects[obj_id] = {
                        'name': 'Unknown',
                        'x': wx_robot,
                        'y': wy_robot,
                        'z': wz_board,
                        'distance': distance,
                        'bbox': (xmin, ymin, xmax, ymax),
                        'center': (int(cx), int(cy)),
                        'type': 'unknown',
                        'area': unknown['area']
                    }
            
            for obj_id, obj_data in current_frame_objects.items():
                if obj_id in detected_objects:
                    old_obj = detected_objects[obj_id]
                    dx = obj_data['x'] - old_obj['x']
                    dy = obj_data['y'] - old_obj['y']
                    distance_moved = (dx**2 + dy**2) ** 0.5
                    
                    if distance_moved >= POSITION_CHANGE_THRESHOLD:
                        obj_data['stable_since'] = now
                        detected_objects[obj_id] = obj_data
                        sent_objects.discard(obj_id)
                    else:
                        obj_data['stable_since'] = old_obj['stable_since']
                        detected_objects[obj_id] = obj_data
                else:
                    obj_data['stable_since'] = now
                    detected_objects[obj_id] = obj_data
            
            disappeared = set(detected_objects.keys()) - set(current_frame_objects.keys())
            for obj_id in disappeared:
                del detected_objects[obj_id]
                sent_objects.discard(obj_id)
            
            last_detection_time = now
            check_and_send_stable_objects()
            
            time.sleep(0.01)
            
        except Exception as e:
            time.sleep(0.1)

def render_thread():
    global current_frame, BOARD_MASK_RESIZED
    
    while streaming:
        if len(frame_queue) == 0:
            time.sleep(0.01)
            continue
        
        try:
            frame = frame_queue[-1].copy()
            
            if board_detected and homography_matrix is not None and BOARD_WIDTH_CM is not None:
                board_corners_cm = np.float32([[0, 0], [BOARD_WIDTH_CM, 0], [BOARD_WIDTH_CM, BOARD_HEIGHT_CM], [0, BOARD_HEIGHT_CM]]).reshape(-1, 1, 2)
                H_inv = np.linalg.inv(homography_matrix)
                board_corners_px = cv2.perspectiveTransform(board_corners_cm, H_inv)
                pts = np.int32(board_corners_px)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                
                if camera_matrix is not None:
                    cx_px = int(camera_matrix[0, 2])
                    cy_px = int(camera_matrix[1, 2])
                    
                    cv2.drawMarker(frame, (cx_px, cy_px), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                    cv2.circle(frame, (cx_px, cy_px), 3, (0, 0, 255), -1)
                    
                    if camera_position_board is not None:
                        cv2.putText(frame, f"Camera Center (px): ({cx_px}, {cy_px})", 
                                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                        cv2.putText(frame, f"Nadir Point (board): ({camera_position_board[0]:.1f}, {camera_position_board[1]:.1f})cm", 
                                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            now = time.time()
            
            for obj_id, obj in detected_objects.items():
                xmin, ymin, xmax, ymax = obj['bbox']
                cx, cy = obj['center']
                stable_time = now - obj['stable_since']
                
                color = (0, 255, 255) if stable_time >= OBJECT_STABILITY_TIME else (255, 150, 0)
                if obj_id in sent_objects: color = (0, 200, 0)
                
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
                cv2.circle(frame, (cx, cy), 5, (0, 255, 255), -1) 
                
                if board_detected and homography_matrix is not None:
                    board_pt = np.array([[[obj['x'] - ROBOT_OFFSET_X, obj['y'] - ROBOT_OFFSET_Y]]], dtype=np.float32)
                    corrected_px = cv2.perspectiveTransform(board_pt, np.linalg.inv(homography_matrix))
                    ccx, ccy = int(corrected_px[0][0][0]), int(corrected_px[0][0][1])
                    
                    cv2.line(frame, (cx, cy), (ccx, ccy), (200, 200, 200), 1)
                    cv2.circle(frame, (ccx, ccy), 5, (255, 0, 255), -1)
                
                lines_to_draw = []
                if board_detected and homography_matrix is not None:
                    if obj_id in sent_objects:
                        lines_to_draw.append(f"{obj['name']} (SENT)")
                    elif stable_time >= OBJECT_STABILITY_TIME:
                        lines_to_draw.append(f"{obj['name']}")
                        lines_to_draw.append(f"Board: ({obj['x'] - ROBOT_OFFSET_X:.1f}, {obj['y'] - ROBOT_OFFSET_Y:.1f})cm")
                        lines_to_draw.append(f"Robot: ({obj['x']:.1f}, {obj['y']:.1f})cm")

                line_height = 18 
                for i, text_line in enumerate(reversed(lines_to_draw)):
                    y_offset = ymin - 10 - (i * line_height)
                    current_line_color = (0, 165, 255) if "Robot:" in text_line else color
                    cv2.putText(frame, text_line, (xmin, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, current_line_color, 1)
            
            cv2.putText(frame, "RED CROSS: Camera Center", (frame.shape[1] - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.putText(frame, "Yellow: Original", (frame.shape[1] - 250, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(frame, "Purple: Corrected", (frame.shape[1] - 250, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
            
            if camera_height_cm is not None:
                height_text = f"Camera Height Above Board: {camera_height_cm - BOARD_Z_OFFSET:.1f} cm"
                cv2.putText(frame, height_text, (10, frame.shape[0] - 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            fps_text = f"FPS: {fps_counter['fps']:.1f}"
            cv2.putText(frame, fps_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            status_text = f"Board: {'DETECTED' if board_detected else 'NOT DETECTED'}"
            status_color = (0, 255, 0) if board_detected else (0, 0, 255)
            cv2.putText(frame, status_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            with frame_lock:
                current_frame = frame
            update_fps()
            time.sleep(0.01)
        except Exception as e:
            time.sleep(0.01)

def fetch_frames():
    threading.Thread(target=capture_thread, daemon=True).start()
    threading.Thread(target=detection_thread, daemon=True).start()
    threading.Thread(target=render_thread, daemon=True).start()
    
    while streaming:
        time.sleep(0.1)

def generate_frames():
    while streaming:
        with frame_lock:
            if current_frame is not None:
                ret, buffer = cv2.imencode('.jpg', current_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)

@app.route('/register', methods=['POST'])
def register():
    global ESP32_IP, ESP8266_IP
    data = request.get_json()
    
    if data.get('device') == 'esp32cam':
        ESP32_IP = data.get('ip')
    elif data.get('device') == 'esp8266':
        ESP8266_IP = data.get('ip')
    
    return jsonify({"status": "ok"})

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/start_stream', methods=['POST'])
def start_stream():
    global streaming
    
    if ESP32_IP is None:
        return jsonify({"status": "error", "message": "ESP32-CAM not registered"}), 400
    
    if not streaming:
        streaming = True
        
        try:
            requests.get(f"http://{ESP32_IP}/start", timeout=1)
        except:
            pass
        
        threading.Thread(target=fetch_frames, daemon=True).start()
    
    return jsonify({"status": "started"})

@app.route('/stop_stream', methods=['POST'])
def stop_stream():
    global streaming, current_frame, detected_objects, sent_objects, current_pick_in_progress
    global homography_matrix, board_detected, BOARD_WIDTH_CM, BOARD_HEIGHT_CM
    global camera_height_cm, camera_position_board
    
    streaming = False
    current_frame = None
    detected_objects = {}
    sent_objects = set()
    current_pick_in_progress = False
    homography_matrix = None
    board_detected = False
    BOARD_WIDTH_CM = None
    BOARD_HEIGHT_CM = None
    camera_height_cm = None
    camera_position_board = None
    
    if ESP32_IP:
        try:
            requests.get(f"http://{ESP32_IP}/stop", timeout=1)
        except:
            pass
    
    return jsonify({"status": "stopped"})

@app.route('/status')
def status():
    return jsonify({
        "streaming": streaming,
        "board_detected": board_detected,
        "fps": round(fps_counter['fps'], 1),
        "objects_detected": len(detected_objects),
        "objects_sent": len(sent_objects),
        "pick_in_progress": current_pick_in_progress,
        "camera_height": round(camera_height_cm, 1) if camera_height_cm else None
    })

@app.route('/pick_success', methods=['POST'])
def pick_success():
    global current_pick_in_progress
    
    data = request.get_json()
    name = data.get('name')
    x = data.get('x', 0) / 10.0
    y = data.get('y', 0) / 10.0
    z = data.get('z', 0) / 10.0
    
    current_pick_in_progress = False
    
    return jsonify({"status": "ok", "message": "Ready for next"})

@app.route('/retry_object', methods=['POST'])
def retry_object():
    global current_pick_in_progress, sent_objects
    
    data = request.get_json()
    name = data.get('name')
    x = data.get('x', 0) / 10.0
    y = data.get('y', 0) / 10.0
    z = data.get('z', 0) / 10.0
    
    obj_id = get_object_id(name, x, y)
    sent_objects.discard(obj_id)
    
    current_pick_in_progress = False
    
    return jsonify({"status": "ok", "message": "Will retry"})

@app.route('/set_threshold', methods=['POST'])
def set_threshold():
    global YOLO_HIGH_CONFIDENCE
    data = request.get_json()
    new_threshold = data.get('threshold', YOLO_HIGH_CONFIDENCE)
    YOLO_HIGH_CONFIDENCE = max(0.5, min(0.99, new_threshold))
    return jsonify({"status": "ok", "threshold": YOLO_HIGH_CONFIDENCE})

if __name__ == '__main__':
    load_camera_parameters()
    load_board_mask()
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)