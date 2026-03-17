#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8
from cv_bridge import CvBridge
import cv2

class StartDetector(Node):
    def __init__(self):
        super().__init__('start_detector')
        
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        self.start_pub = self.create_publisher(UInt8, '/game_started', 10)
        
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict)
        
        self.game_started = False
        self.marker_count_history = []
        
    def image_callback(self, msg):
        if self.game_started:
            return
            
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        current_count = len(ids) if ids is not None else 0
        self.marker_count_history.append(current_count)
        
        # Храним последние 10 кадров
        if len(self.marker_count_history) > 10:
            self.marker_count_history.pop(0)
        
        # Проверяем переход 11 -> 12
        if len(self.marker_count_history) == 10:
            prev_avg = sum(self.marker_count_history[:-1]) / 9
            if prev_avg > 10.5 and current_count == 12:
                self.game_started = True
                self.start_pub.publish(UInt8(data=1))
                self.get_logger().info("GAME STARTED!")

def main(args=None):
    rclpy.init(args=args)
    node = StartDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
