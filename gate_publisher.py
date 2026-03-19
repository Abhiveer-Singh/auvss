#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2 as cv
import numpy as np

# =========================
# Constants
# =========================
COLOR_RANGES = {
    "blue":   [(np.array([90, 80, 40]), np.array([140, 255, 255]))],
    "red":    [(np.array([0, 80, 40]), np.array([10, 255, 255])),
               (np.array([165, 80, 40]), np.array([180, 255, 255]))],
    "green":  [(np.array([35, 60, 40]), np.array([85, 255, 255]))],
    "yellow": [(np.array([15, 80, 40]), np.array([40, 255, 255]))]
}

KNOWN_DISTANCE = 150.0      # cm
REAL_GATE_WIDTH = 100.0     # cm
FOCAL_LENGTH = None
distance_buffer = []

class GatePublisher(Node):
    def __init__(self):
        super().__init__('gate_publisher')
        self.publisher_ = self.create_publisher(String, 'gate_info', 10)
        
        # Try multiple camera indices
        self.cap = None
        for i in range(4):
            self.cap = cv.VideoCapture(i)
            if self.cap.isOpened():
                self.get_logger().info(f'Camera {i} opened successfully')
                break
        if self.cap is None:
            self.get_logger().error('No camera found!')
            return
        
        # TWO WINDOWS: Clean feed + Processed frame
        cv.namedWindow("🖥️ CAMERA FEED - CLEAN (Press Q)", cv.WINDOW_NORMAL)
        cv.moveWindow("🖥️ CAMERA FEED - CLEAN (Press Q)", 50, 50)
        cv.resizeWindow("🖥️ CAMERA FEED - CLEAN (Press Q)", 640, 480)
        
        cv.namedWindow("🎯 GATE DETECTION (Press Q)", cv.WINDOW_NORMAL)
        cv.moveWindow("🎯 GATE DETECTION (Press Q)", 700, 50)
        cv.resizeWindow("🎯 GATE DETECTION (Press Q)", 800, 600)
        
        self.get_logger().info('🖥️ 2 Camera windows opening... Press Q in ANY window to quit')
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        # SAVE CLEAN FRAME FIRST (for separate window)
        clean_frame = frame.copy()

        # Preprocessing (only for detection frame)
        frame = cv.bilateralFilter(frame, d=9, sigmaColor=75, sigmaSpace=75)

        # CLAHE enhancement
        lab = cv.cvtColor(frame, cv.COLOR_BGR2LAB)
        l, a, b = cv.split(lab)
        clahe = cv.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        l = clahe.apply(l)
        lab = cv.merge((l, a, b))
        frame = cv.cvtColor(lab, cv.COLOR_LAB2BGR)

        # HSV equalization
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        h, s, v = cv.split(hsv)
        v = cv.equalizeHist(v)
        hsv = cv.merge((h, s, v))

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (5, 5), 0)

        h, w, _ = frame.shape
        frame_cx, frame_cy = w // 2, h // 2
        detections = {}
        combined_mask = np.zeros((h, w), dtype=np.uint8)

        # Detect all colors - FIXED VERSION
        for color_name, ranges in COLOR_RANGES.items():
            mask = None
            for lower, upper in ranges:
                temp_mask = cv.inRange(hsv, lower, upper)
                mask = temp_mask if mask is None else cv.bitwise_or(mask, temp_mask)

            edges = cv.Canny(gray, 50, 150)

            # Morphological cleanup
            kernel = np.ones((5, 5), np.uint8)
            mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
            mask = cv.morphologyEx(mask, cv.MORPH_DILATE, kernel)

            combined_mask = cv.bitwise_or(combined_mask, mask)

            # Find contours for this color
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            if contours:
                gate = max(contours, key=cv.contourArea)
                area = cv.contourArea(gate)
                if area > 500:
                    x, y, bw, bh = cv.boundingRect(gate)
                    cx, cy = x + bw // 2, y + bh // 2

                    aspect_ratio = bw / float(bh)
                    confidence = area / float(bw * bh)

                    if 0.8 < aspect_ratio < 1.2 and confidence > 0.6:
                        cv.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                        cv.putText(frame, f"{color_name} ({confidence:.2f})", (x, y - 10),
                                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                        detections[color_name] = {"cx": cx, "cy": cy, "bw": bw, "bh": bh, "confidence": confidence}

        # Navigation logic
        command = "SEARCH"
        smoothed_distance = 0.0
        offset_x = 0.0
        offset_y = 0.0
        gate_cx, gate_cy = 0, 0

        # Gate prioritization
        preferred_order = ["green", "red", "blue", "yellow"]
        ASSIGNED_COLOR = None
        for color in preferred_order:
            if color in detections:
                ASSIGNED_COLOR = color
                break

        # Process assigned gate
        if ASSIGNED_COLOR and ASSIGNED_COLOR in detections:
            target = detections[ASSIGNED_COLOR]
            gate_cx, gate_cy = target["cx"], target["cy"]
            cv.circle(frame, (gate_cx, gate_cy), 5, (0, 0, 255), -1)

            # Distance estimation
            bw = target["bw"]
            bh = target["bh"]
            avg_size = (bw + bh) / 2

            if FOCAL_LENGTH is None:
                FOCAL_LENGTH = (avg_size * KNOWN_DISTANCE) / REAL_GATE_WIDTH

            distance_cm = (REAL_GATE_WIDTH * FOCAL_LENGTH) / avg_size

            # Smooth distance
            distance_buffer.append(distance_cm)
            if len(distance_buffer) > 5:
                distance_buffer.pop(0)
            smoothed_distance = sum(distance_buffer) / len(distance_buffer)

            cv.putText(frame, f"Distance: {int(smoothed_distance)} cm", (10, 60),
                       cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # Alignment
            dx = gate_cx - frame_cx
            dy = gate_cy - frame_cy
            offset_x = dx / float(w)
            offset_y = dy / float(h)

            cv.putText(frame, f"Offset X: {offset_x:.2f}", (10, 150),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv.putText(frame, f"Offset Y: {offset_y:.2f}", (10, 180),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            # Command logic
            if abs(dx) > abs(dy):
                if dx > 40:
                    command = "MOVE RIGHT"
                elif dx < -40:
                    command = "MOVE LEFT"
                else:
                    command = "CENTERED"
            else:
                if dy > 30:
                    command = "MOVE UP"
                elif dy < -30:
                    command = "MOVE DOWN"
                else:
                    command = "CENTERED"

        # Status messages
        if "green" in detections and "red" in detections:
            cv.putText(frame, "Prioritizing GREEN over RED", (10, 220),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        elif ASSIGNED_COLOR == "green":
            cv.putText(frame, "Going towards GREEN gate", (10, 220),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        elif ASSIGNED_COLOR == "red":
            cv.putText(frame, "Going towards RED gate", (10, 220),
                       cv.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Crosshair
        cv.line(frame, (frame_cx, 0), (frame_cx, h), (255, 255, 255), 1)
        cv.line(frame, (0, frame_cy), (w, frame_cy), (255, 255, 255), 1)

        cv.putText(frame, f"COMMAND: {command}", (10, 30),
                   cv.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # SHOW BOTH WINDOWS
        cv.imshow("🖥️ CAMERA FEED - CLEAN (Press Q)", clean_frame)
        cv.imshow("🎯 GATE DETECTION (Press Q)", frame)

        # PUBLISH DATA
        msg_data = (f"color:{ASSIGNED_COLOR},dist:{smoothed_distance:.1f},"
                   f"ox:{offset_x:.2f},oy:{offset_y:.2f},cx:{gate_cx},cy:{gate_cy},cmd:{command}")
        
        msg = String()
        msg.data = msg_data
        self.publisher_.publish(msg)

        # Check for Q in ANY window
        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Q pressed - stopping')

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
