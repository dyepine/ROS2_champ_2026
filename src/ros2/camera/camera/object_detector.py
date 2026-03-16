Создадим новый пакет для детекции объектов
Нужен узел, который будет определять игровые объекты (кубики, цилиндры) и их стоимость:

object_detector.py:

python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header, Int16
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Параметры
        self.team = os.getenv("TEAM", "1")
        
        # Подписка на изображение
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        
        # Публикация позиций объектов
        self.object_pub = self.create_publisher(
            PoseStamped, '/object_poses', 10
        )
        
        # Публикация счета (для тестирования)
        self.score_pub = self.create_publisher(
            Int16, '/detected_score', 10
        )
        
        self.bridge = CvBridge()
        
        # Загружаем словарь ArUco для детекции объектов
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        
        # Стоимость объектов по ID маркеров (будет загружаться из конфига)
        self.object_values = {}
        self.load_object_values()
        
        self.get_logger().info(f"Object detector initialized for team {self.team}")
    
    def load_object_values(self):
        """Загружает стоимость объектов из конфигурационного файла"""
        try:
            config_path = f"/ros2_ws/src/strategy/config/object_values_team{self.team}.yaml"
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                self.object_values = config.get('object_values', {})
            self.get_logger().info(f"Loaded object values: {self.object_values}")
        except Exception as e:
            self.get_logger().warn(f"Could not load object values: {e}")
            # Значения по умолчанию
            self.object_values = {
                'cube_red': 10,
                'cube_blue': 20,
                'cylinder': 15,
                'duck': -5
            }
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # Детектируем ArUco маркеры на объектах
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        
        if ids is not None:
            for i, marker_id in enumerate(ids[0]):
                # Вычисляем центр маркера в пикселях
                corner = corners[i][0]
                center_x = int(np.mean(corner[:, 0]))
                center_y = int(np.mean(corner[:, 1]))
                
                # TODO: Здесь нужно преобразовать пиксельные координаты в мировые
                # Для этого нужна калибровка камеры и знание высоты
                
                # Временная реализация - публикуем с заглушкой
                pose_msg = PoseStamped()
                pose_msg.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
                pose_msg.pose.position.x = float(marker_id) * 0.1  # Заглушка
                pose_msg.pose.position.y = 0.0
                pose_msg.pose.position.z = 0.0
                
                self.object_pub.publish(pose_msg)
                
                # Логируем детекцию
                self.get_logger().info(f"Detected object with marker ID: {marker_id}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
