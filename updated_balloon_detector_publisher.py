#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

# =========================
# HSV COLOR RANGES
# =========================
COLOR_RANGES = {
    "Red": [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255]))
    ],
    "Green": [
        (np.array([36, 100, 100]), np.array([86, 255, 255]))
    ],
    "Blue": [
        (np.array([94, 80, 2]), np.array([126, 255, 255]))
    ],
    "Yellow": [
        (np.array([25, 100, 100]), np.array([35, 255, 255]))
    ]
}

BGR_COLORS = {
    "Red": (0, 0, 255),
    "Green": (0, 255, 0),
    "Blue": (255, 0, 0),
    "Yellow": (0, 255, 255)
}

def underwater_white_balance(img):
    img = img.astype(np.float32)
    b, g, r = cv2.split(img)
    r_gain = np.mean(g) / (np.mean(r) + 1e-6)
    b_gain = np.mean(g) / (np.mean(b) + 1e-6)
    r *= r_gain
    b *= b_gain
    balanced = cv2.merge((b, g, r))
    return np.clip(balanced, 0, 255).astype(np.uint8)

class BalloonDetectionPublisher(Node):
    def __init__(self):
        super().__init__('balloon_detection_publisher')
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'ballon_detection/camera/image_raw', 10)
        self.detection_pub = self.create_publisher(String, 'balloon_detection/detect_status', 10)
        
        self.bridge = CvBridge()
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera Feed", 800, 600)
        cv2.namedWindow("Detection Mask", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Detection Mask", 400, 300)
        
        self.timer = self.create_timer(0.033, self.timer_callback)  # ~30 FPS
        self.get_logger().info('Balloon Detection Publisher started - FORMAT: COLOR,dx,dy,offset,instruction')
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        h, w = frame.shape[:2]
        frame_center = (w // 2, h // 2)
        
        # Draw crosshair
        cv2.line(frame, (frame_center[0], 0), (frame_center[0], h), (255, 255, 255), 1)
        cv2.line(frame, (0, frame_center[1]), (w, frame_center[1]), (255, 255, 255), 1)
        
        # Preprocessing
        frame_processed = underwater_white_balance(frame)
        frame_processed = cv2.GaussianBlur(frame_processed, (5, 5), 0)
        hsv = cv2.cvtColor(frame_processed, cv2.COLOR_BGR2HSV)
        
        mask_display = np.zeros((h, w), dtype=np.uint8)
        detection_msg = ""
        detected_color = ""
        dx_val = 0.0
        dy_val = 0.0
        offset_val = 0.0
        instruction_val = "No Detection"
        
        # Color detection - COLOR IS ALWAYS INCLUDED
        for color_name, ranges in COLOR_RANGES.items():
            mask = None
            for lower, upper in ranges:
                temp_mask = cv2.inRange(hsv, lower, upper)
                mask = temp_mask if mask is None else cv2.bitwise_or(mask, temp_mask)
            
            # Morphology
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)
            
            mask_display = cv2.bitwise_or(mask_display, mask)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:
                    x, y, bw, bh = cv2.boundingRect(cnt)
                    cx, cy = x+bw//2, y+bh//2
                    
                    # CALCULATE ALL VALUES
                    dx_val = cx - frame_center[0]
                    dy_val = cy - frame_center[1]
                    offset_val = float(np.sqrt(dx_val**2 + dy_val**2))
                    
                    # Movement instruction
                    instruction_val = "Centered"
                    if abs(dx_val) > 40 or abs(dy_val) > 30:
                        h_dir = "Right" if dx_val > 0 else "Left" if dx_val < 0 else ""
                        v_dir = "Down" if dy_val > 0 else "Up" if dy_val < 0 else ""
                        instruction_val = f"Move {h_dir} {v_dir}".strip()
                    
                    # **COLOR IS ALWAYS PUBLISHED FIRST**
                    detected_color = color_name
                    
                    # Draw bounding box
                    cv2.rectangle(frame, (x,y), (x+bw, y+bh), BGR_COLORS[color_name], 2)
                    cv2.putText(frame, f"{color_name}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, BGR_COLORS[color_name], 2)
                    cv2.putText(frame, instruction_val, (x, y+bh+20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # **PUBLISH MESSAGE WITH COLOR FIRST**
                    detection_msg = f"{color_name},{dx_val:.1f},{dy_val:.1f},{offset_val:.1f},{instruction_val}"
                    
                    # Log to console
                    self.get_logger().info(f"PUBLISHED: {detection_msg}")
                    
                    break  # Use first valid detection
            if detection_msg:  # Stop after first detection
                break
        
        # Always publish something (even if no detection)
        if not detection_msg:
            detection_msg = f"NoDetection,0.0,0.0,0.0,No Detection"
        
        # Publish images and detection data
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            
            msg = String()
            msg.data = detection_msg
            self.detection_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Publish error: {str(e)}')
        
        # Display
        cv2.imshow("Camera Feed", frame)
        cv2.imshow("Detection Mask", mask_display)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Quit signal received')
    
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BalloonDetectionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
