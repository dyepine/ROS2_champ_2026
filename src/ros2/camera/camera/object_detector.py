#!/usr/bin/env python3
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
import math

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        
        # Параметры
        self.team = os.getenv("TEAM", "1")
        self.camera_height = 1.5  # Высота камеры над полем (1500 мм из правил)
        
        # Загружаем калибровку камеры
        self.load_camera_calibration()
        
        # Подписка на изображение
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )
        
        # Публикация позиций объектов
        self.object_pub = self.create_publisher(
            PoseStamped, '/object_poses', 10
        )
        
        # Публикация маркеров для визуализации
        self.marker_pub = self.create_publisher(
            PoseStamped, '/object_markers', 10
        )
        
        self.bridge = CvBridge()
        
        # Загружаем словарь ArUco для детекции объектов
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)  # Тот же словарь, что и при калибровке
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        
        # Стоимость объектов (загружается из конфига)
        self.object_values = {}
        self.load_object_values()
        
        self.get_logger().info(f"Object detector initialized for team {self.team}")
        self.get_logger().info(f"Camera matrix: fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}")
    
    def load_camera_calibration(self):
        """Загружает калибровочные данные камеры"""
        try:
            # Путь к файлу калибровки
            script_dir = os.path.dirname(os.path.abspath(__file__))
            package_dir = os.path.dirname(script_dir)
            config_path = os.path.join(package_dir, 'config', 'camera_calibration_config.yaml')
            
            with open(config_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            self.camera_matrix = np.array(calib_data['camera_matrix'], dtype=np.float32)
            self.dist_coeffs = np.array(calib_data['dist_coeff'], dtype=np.float32)
            self.image_width = calib_data['image_width']
            self.image_height = calib_data['image_height']
            
            self.get_logger().info(f"Loaded camera calibration with error: {calib_data['reprojection_error']:.3f} px")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load camera calibration: {e}")
            # Fallback to approximate matrix
            self.camera_matrix = np.array([
                [1000, 0, 640],
                [0, 1000, 480],
                [0, 0, 1]
            ], dtype=np.float32)
            self.dist_coeffs = np.zeros(5)
    
    def load_object_values(self):
        """Загружает стоимость объектов из конфигурационного файла"""
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            package_dir = os.path.dirname(script_dir)
            config_path = os.path.join(package_dir, 'config', f'object_values_team{self.team}.yaml')
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                self.object_values = config.get('object_values', {})
                
            self.get_logger().info(f"Loaded {len(self.object_values)} object values")
        except Exception as e:
            self.get_logger().warn(f"Could not load object values: {e}")
            self.object_values = {}
    
    def pixel_to_world(self, pixel_x, pixel_y, marker_size_world=0.046):
        """
        Преобразует пиксельные координаты в мировые (X, Y на плоскости пола)
        Использует калибровку камеры и знание высоты
        
        Args:
            pixel_x, pixel_y: координаты центра маркера в пикселях
            marker_size_world: реальный размер маркера в метрах (по умолчанию 46 мм)
        
        Returns:
            (x, y) координаты в метрах относительно камеры
        """
        # Формируем точку в пиксельных координатах
        point_2d = np.array([[pixel_x, pixel_y]], dtype=np.float32)
        
        # Неискажаем точку (убираем дисторсию)
        undistorted = cv2.undistortPoints(
            point_2d, 
            self.camera_matrix, 
            self.dist_coeffs,
            P=self.camera_matrix
        )
        
        # Нормализованные координаты (x/z, y/z) в системе камеры
        norm_x = undistorted[0][0][0]
        norm_y = undistorted[0][0][1]
        
        # Предполагаем, что объект лежит на полу (Z=0 в мировой системе)
        # Луч от камеры до пола: z_camera = -self.camera_height (камера смотрит вниз)
        
        # В системе координат камеры: ось Z направлена вперед, Y вниз
        # Но нам нужно преобразовать
        
        # Упрощенная модель: предполагаем, что камера смотрит прямо вниз
        # Тогда масштаб = высота / фокусное расстояние
        scale = self.camera_height / self.camera_matrix[1, 1]
        
        world_x = norm_x * scale
        world_y = norm_y * scale
        
        return world_x, world_y
    
    def estimate_pose_pnp(self, corners, marker_size=0.046):
        """
        Оценивает позу маркера с помощью solvePnP
        
        Args:
            corners: углы маркера (4 точки)
            marker_size: размер маркера в метрах
        
        Returns:
            rvec, tvec или None если не удалось
        """
        # 3D точки углов маркера
        obj_points = np.array([
            [-marker_size/2,  marker_size/2, 0],
            [ marker_size/2,  marker_size/2, 0],
            [ marker_size/2, -marker_size/2, 0],
            [-marker_size/2, -marker_size/2, 0]
        ], dtype=np.float32)
        
        # 2D точки на изображении
        img_points = corners.reshape(4, 2).astype(np.float32)
        
        # Решаем PnP
        success, rvec, tvec = cv2.solvePnP(
            obj_points, 
            img_points, 
            self.camera_matrix, 
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if success:
            return rvec, tvec
        return None, None
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
        
        # Улучшаем изображение
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        
        # Детектируем маркеры
        corners, ids, _ = self.detector.detectMarkers(enhanced)
        
        if ids is not None and len(ids) > 0:
            self.get_logger().info(f"Detected {len(ids)} markers")
            
            for i, (marker_id, corner) in enumerate(zip(ids.flatten(), corners)):
                # Вычисляем центр маркера
                center_x = int(np.mean(corner[0][:, 0]))
                center_y = int(np.mean(corner[0][:, 1]))
                
                # Метод 1: Преобразование пиксель -> мир (упрощенное)
                world_x, world_y = self.pixel_to_world(center_x, center_y)
                
                # Метод 2: PnP оценка (более точная)
                rvec, tvec = self.estimate_pose_pnp(corner[0])
                
                # Получаем стоимость объекта
                value = self.object_values.get(int(marker_id), 0)
                
                # Публикуем позицию
                pose_msg = PoseStamped()
                pose_msg.header = Header(
                    frame_id='map', 
                    stamp=self.get_clock().now().to_msg()
                )
                
                if tvec is not None:
                    # Используем PnP результат
                    # tvec - это позиция маркера в системе камеры
                    # Нам нужно преобразовать в мировую систему
                    # Пока используем упрощенно
                    pose_msg.pose.position.x = tvec[0][0]
                    pose_msg.pose.position.y = tvec[1][0]
                    pose_msg.pose.position.z = 0.0
                else:
                    # Используем упрощенное преобразование
                    pose_msg.pose.position.x = world_x
                    pose_msg.pose.position.y = world_y
                    pose_msg.pose.position.z = 0.0
                
                self.object_pub.publish(pose_msg)
                
                # Логируем для отладки
                self.get_logger().info(
                    f"Marker {marker_id}: pos=({pose_msg.pose.position.x:.3f}, {pose_msg.pose.position.y:.3f}), "
                    f"value={value}"
                )
                
                # Рисуем на изображении (для отладки)
                cv2.aruco.drawDetectedMarkers(cv_image, [corner], np.array([[marker_id]]))
                cv2.putText(
                    cv_image, 
                    f"ID:{marker_id} V:{value}", 
                    (center_x - 20, center_y - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, (0, 255, 0), 2
                )
            
            # Показываем изображение (опционально)
            cv2.imshow('Object Detection', cv2.resize(cv_image, (960, 540)))
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Object detector stopped")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
