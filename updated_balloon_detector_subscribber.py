#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import csv
import cv2 
from datetime import datetime

class BalloonDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('balloon_detection_subscriber')
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.detection_sub = self.create_subscription(
            String, '/balloon_detection', self.detection_callback, 10)
        
        self.bridge = CvBridge()
        
        # Storage for latest data
        self.latest_detection = None
        self.csv_file = None
        self.writer = None
        
        # Open CSV for logging
        self.open_csv()
        
        cv2.namedWindow("ROS Camera Feed", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ROS Camera Feed", 800, 600)
        
        self.get_logger().info('Balloon Detection Subscriber started')
        self.get_logger().info('Detection format: color,dx,dy,offset,instruction')
    
    def open_csv(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"balloon_detection_{timestamp}.csv"
        self.csv_file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['timestamp', 'color', 'dx', 'dy', 'offset', 'instruction'])
        self.get_logger().info(f'Logging to {filename}')
    
    def detection_callback(self, msg):
        try:
            data = msg.data.split(',')
            if len(data) == 5:
                self.latest_detection = {
                    'color': data[0],
                    'dx': float(data[1]),
                    'dy': float(data[2]),
                    'offset': float(data[3]),
                    'instruction': data[4]
                }
                
                timestamp = datetime.now().isoformat()
                self.writer.writerow([timestamp, *data])
                self.csv_file.flush()
                
                self.get_logger().info(
                    f"Balloon: {data[0]} | dx:{data[1]:.1f} dy:{data[2]:.1f} "
                    f"offset:{data[3]:.1f} | {data[4]}"
                )
        except Exception as e:
            self.get_logger().error(f'Detection parse error: {str(e)}')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Overlay detection info
            if self.latest_detection:
                info = self.latest_detection
                text = (f"{info['color']} Balloon | dx:{info['dx']:.1f} dy:{info['dy']:.1f} "
                       f"offset:{info['offset']:.1f} | {info['instruction']}")
                cv2.putText(cv_image, text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            cv2.imshow("ROS Camera Feed", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Image callback error: {str(e)}')
    
    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BalloonDetectionSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
