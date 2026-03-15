import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
import os
from cv2 import aruco

def read_config(config_path):
    with open(config_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

class Camera:
    def __init__(self, config_path: str):
        self.config = read_config(config_path)
        self.camera_matrix = np.array(self.config["camera_matrix"], dtype=np.float64)
        self.dist_coefs = np.array(self.config["dist_coeff"], dtype=np.float64)
        self.newcameramatrix, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.dist_coefs, (1600, 896), 0.5, (1600, 896)
        )
        self.robot_id = self.config["robot_id"]

        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        
        # Инициализация ChArUco доски с параметрами из image.png
        self.charuco_board = cv2.aruco.CharucoBoard(
            (5, 7),  # (columns, rows) - 5x7 клеток
            0.25,    # размер клетки в метрах (250 мм)
            0.175,   # размер маркера в метрах (уменьшил до 70% от клетки для лучшего детектирования)
            self.arucoDict
        )
        self.charuco_detector = cv2.aruco.CharucoDetector(self.charuco_board)	
        
        # Добавляем параметры поля из правил
        self.field_width = 1.75  # 1750 мм
        self.field_height = 1.25  # 1250 мм
        self.cell_size = 0.25     # 250 мм
        
        # Координаты угловых зон хранения (коробки 250x250 мм с бортиками)
        self.storage_zones = {
            'team1': np.array([-self.field_width/2 + self.cell_size/2, 
                               self.field_height/2 - self.cell_size/2, 0.0]),  # Левый верхний угол
            'team2': np.array([self.field_width/2 - self.cell_size/2, 
                              self.field_height/2 - self.cell_size/2, 0.0])   # Правый верхний угол
        }
        
        # Координаты стартовых зон (квадраты A и B)
        self.start_zones = {
            'A': np.array([-self.field_width/2 + self.cell_size*1.5, 
                          -self.field_height/2 + self.cell_size*1.5, 0.0]),
            'B': np.array([self.field_width/2 - self.cell_size*1.5, 
                          -self.field_height/2 + self.cell_size*1.5, 0.0])
        }

        team = os.getenv("TEAM")

        if team == "1":
            self.colour_range = range(1,6)
            # ID угловых маркеров для команды 1 (используем Start Id 0 из image.png)
            self.corner_markers = [0, 1, 2, 3]  # Углы поля
            
            self.RotSideDict = {
                55: R.from_euler('y', 90, degrees=True).as_matrix(),    # Front: x down, z forward
                56: R.from_euler('x', 90, degrees=True).as_matrix(),    # Right: x forward, z right
                57: R.from_euler('y', -90, degrees=True).as_matrix(),   # Rear: x up, z backward
                58: R.from_euler('x', -90, degrees=True).as_matrix(),   # Left: x forward, z left
                127: np.eye(3),   # Top: no rotation (example)
                126: np.eye(3)    # Side: x forward, z right (example)
            }
            self.TvecSideDict = {
                55: np.array([-0.05, 0.0, 0.055]),  # From 55 to self.robot_id
                56: np.array([0.0, 0.05, 0.055]),   # From 56 to self.robot_id
                57: np.array([0.05, 0.0, 0.055]),   # From 57 to self.robot_id
                58: np.array([0.0, -0.05, 0.055]),  # From 58 to self.robot_id
                127: np.array([-0.043, 0.15, 0.135]),   # From 127 to self.robot_id 
                126: np.array([-0.043, -0.15, 0.135])   # From 126 to self.robot_id
            }

        else:
            self.colour_range = range(6,11)
            # ID угловых маркеров для команды 2
            self.corner_markers = [4, 5, 6, 7]  # Углы поля

            self.RotSideDict = {
                74: R.from_euler('y', 90, degrees=True).as_matrix(),    # Front: x down, z forward
                75: R.from_euler('x', 90, degrees=True).as_matrix(),    # Right: x forward, z right
                76: R.from_euler('y', -90, degrees=True).as_matrix(),   # Rear: x up, z backward
                77: R.from_euler('x', -90, degrees=True).as_matrix(),   # Left: x forward, z left
                127: np.eye(3),   # Top: no rotation (example)
                126: np.eye(3)    # Side: x forward, z right (example)
            }
            self.TvecSideDict = {
                74: np.array([-0.05, 0.0, 0.055]),  # From 55 to self.robot_id
                75: np.array([0.0, 0.05, 0.055]),   # From 56 to self.robot_id
                76: np.array([0.05, 0.0, 0.055]),   # From 57 to self.robot_id
                77: np.array([0.0, -0.05, 0.055]),  # From 58 to self.robot_id
                127: np.array([-0.043, 0.15, 0.135]),   # From 127 to self.robot_id 
                126: np.array([-0.043, -0.15, 0.135])   # From 126 to self.robot_id
            }

        # Обновляем координаты полевых маркеров (4 угла ChArUco доски)
        self.field_markers = {
            0: np.array([-self.field_width/2, self.field_height/2, 0.0]),   # Верхний левый
            1: np.array([self.field_width/2, self.field_height/2, 0.0]),    # Верхний правый
            2: np.array([-self.field_width/2, -self.field_height/2, 0.0]),  # Нижний левый
            3: np.array([self.field_width/2, -self.field_height/2, 0.0])    # Нижний правый
        }

        self.last_tmatrix = None
        self.last_center = None
        self.initialized = False
        self.last_img = None
        
    def get_pose_from_charuco(self, img):
        """
        Определяет позицию камеры относительно ChArUco доски
        Возвращает: (успех, rvec, tvec)
        """
        if img is None:
            return False, None, None
            
        img_prepared = self.prepare_image(img)
        
        # Детектируем ChArUco доску
        charuco_corners, charuco_ids, marker_corners, marker_ids = \
            self.charuco_detector.detectBoard(img_prepared)
        
        if charuco_corners is not None and len(charuco_corners) > 3:
            # Оцениваем позу камеры относительно доски
            retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, self.charuco_board,
                self.camera_matrix, self.dist_coefs, None, None
            )
            if retval:
                return True, rvec, tvec
        
        return False, None, None
        
    def prepare_image(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Улучшаем контраст для лучшего детектирования маркеров
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray)
        return gray

    def detect_markers(self, img):
        img_prepared = self.prepare_image(img)
        corners, ids, _ = self.detector.detectMarkers(img_prepared)
        if ids is not None:
            ids = list(map(lambda x: x[0], ids))
        return ids, corners
	
    def t_matrix_building(self, ids, corners, img=None):
        """
        Определяет положение камеры относительно поля
        Использует ChArUco доску для большей точности
        """
        if img is not None:
            self.last_img = img
            
        # Пробуем сначала ChArUco (более точно)
        if hasattr(self, 'last_img') and self.last_img is not None:
            success, rvec, tvec = self.get_pose_from_charuco(self.last_img)
            
            if success:
                tmatrix = cv2.Rodrigues(rvec)[0]
                center = tvec.flatten()
                self.last_tmatrix = tmatrix
                self.last_center = center
                self.initialized = True
                return tmatrix, center
            else:
                print("ChArUco board not detected, falling back to markers")
    
        # Fallback на отдельные маркеры если ChArUco не найден
        return self._fallback_t_matrix_building(ids, corners)

    def _fallback_t_matrix_building(self, ids, corners):
        if not ids or not any(marker_id in self.field_markers for marker_id in ids):
            return self.last_tmatrix, self.last_center

        field_corners = []
        field_ids = []
        for i, marker_id in enumerate(ids):
            if marker_id in self.field_markers:
                field_corners.append(corners[i][0])
                field_ids.append(marker_id)

        if not field_ids:
            return self.last_tmatrix, self.last_center

        print(f"Visible field markers: {len(field_ids)}")

        marker_size = 0.1  # Size of field markers
        object_points = []
        image_points = []
        for mid, corners in zip(field_ids, field_corners):
            obj_pts = np.array([
                [-marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, marker_size / 2, 0],
                [marker_size / 2, -marker_size / 2, 0],
                [-marker_size / 2, -marker_size / 2, 0]
            ], dtype=np.float32) + self.field_markers[mid]
            object_points.extend(obj_pts)
            image_points.extend(corners)

        object_points = np.array(object_points, dtype=np.float32)
        image_points = np.array(image_points, dtype=np.float32)

        if not self.initialized and len(field_ids) >= 3:
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, self.camera_matrix, self.dist_coefs,
                flags=cv2.SOLVEPNP_SQPNP
            )
            if success:
                tmatrix = cv2.Rodrigues(rvec)[0]
                center = tvec.flatten()
                self.last_tmatrix = tmatrix
                self.last_center = center
                self.initialized = True
                print(f"Initialized tmatrix:\n{tmatrix}\ncenter: {center}")
            else:
                print("Failed to initialize with solvePnP")
                return self.last_tmatrix, self.last_center
        elif self.initialized and len(field_ids) >= 1:
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, self.camera_matrix, self.dist_coefs,
                flags=cv2.SOLVEPNP_ITERATIVE, useExtrinsicGuess=True,
                rvec=cv2.Rodrigues(self.last_tmatrix)[0], tvec=self.last_center
            )
            if success:
                center = tvec.flatten()
                self.last_center = center
            else:
                print("Failed to update center with solvePnP")
            tmatrix = self.last_tmatrix
        else:
            return self.last_tmatrix, self.last_center

        return tmatrix, center

    def estimate_robot_pose(self, ids, corners, tmatrix, center, is_our_robot=True):
        if not ids or tmatrix is None or center is None:
            return None, None, None

        robot_corners = []
        robot_ids = []

        # Define enemy colour range based on our colour range
        our_colour_range = self.colour_range
        enemy_colour_range = range(6, 11) if our_colour_range == range(1, 6) else range(1, 6)

        for i, marker_id in enumerate(ids):
            if is_our_robot:
                # Include markers in our colour range or in RotSideDict (our robot's markers)
                if marker_id in our_colour_range or marker_id in self.RotSideDict:
                    robot_corners.append(corners[i][0])
                    robot_ids.append(marker_id)
            else:
                # Include markers in enemy colour range
                if marker_id in enemy_colour_range:
                    robot_corners.append(corners[i][0])
                    robot_ids.append(marker_id)

        if not robot_ids:
            return None, None, None

        print(f"Visible robot markers: {len(robot_ids)}")

        object_points = []
        image_points = []
        for mid, corners in zip(robot_ids, robot_corners):
            # Set marker size based on ID
            if mid in [127, 126]:
                marker_length = 0.085  # 8.5 cm for markers 127 and 126
            elif 1 <= mid <= 10:
                marker_length = 0.07   # 7 cm for enemy markers
            else:
                marker_length = 0.05   # 5 cm for other markers (55-58, 74-77, self.robot_id)
            
            obj_pts = np.array([
                [-marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, marker_length / 2, 0],
                [marker_length / 2, -marker_length / 2, 0],
                [-marker_length / 2, -marker_length / 2, 0]
            ], dtype=np.float32)
            if mid in self.RotSideDict:
                rot_side = self.RotSideDict[mid]
                tvec_side = self.TvecSideDict[mid]
                obj_pts = np.dot(obj_pts, rot_side.T) - tvec_side  # Transformation to self.robot_id system
            object_points.extend(obj_pts)
            image_points.extend(corners)

        object_points = np.array(object_points, dtype=np.float32)
        image_points = np.array(image_points, dtype=np.float32)

        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, self.camera_matrix, self.dist_coefs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not success:
            print("Failed to estimate robot pose with solvePnP")
            return None, None, None

        tvec_cam = tvec.flatten()
        rot_matrix = cv2.Rodrigues(rvec)[0]

        robot_tvec = np.dot(tmatrix.T, tvec_cam - center)
        robot_rot_matrix = np.dot(tmatrix.T, rot_matrix)
        quat = R.from_matrix(robot_rot_matrix).as_quat()

        # Вычисляем ковариацию (упрощенно)
        # В реальности нужно использовать Якобиан, но для начала можно использовать константы
        if is_our_robot:
            # Для своего робота - меньшая ковариация
            cov = np.diag([0.0002, 0.0002, 0.0002, 0.001, 0.001, 0.0002])
        else:
            # Для врага - большая ковариация
            cov = np.diag([0.005, 0.005, 0.003, 0.001, 0.001, 0.005])

        return robot_tvec, quat, cov

    def robots_tracking(self, img):
        ids, corners = self.detect_markers(img)
        # Передаем img в t_matrix_building для ChArUco
        tmatrix, center = self.t_matrix_building(ids, corners, img)

        our_tvec, our_quat, our_cov = self.estimate_robot_pose(ids, corners, tmatrix, center, is_our_robot=True)
        enemy_tvec, enemy_quat, enemy_cov = self.estimate_robot_pose(ids, corners, tmatrix, center, is_our_robot=False)

        if our_tvec is not None:
            print(f"Our robot: x={our_tvec[0]:.3f}, y={our_tvec[1]:.3f}, z={our_tvec[2]:.3f}")
        if enemy_tvec is not None:
            print(f"Enemy robot: x={enemy_tvec[0]:.3f}, y={enemy_tvec[1]:.3f}, z={enemy_tvec[2]:.3f}")

        return our_tvec, our_quat, enemy_tvec, enemy_quat, our_cov, enemy_cov
    
    def check_object_in_storage_zone(self, object_position, team):
        """
        Проверяет, находится ли объект в зоне хранения команды
        Возвращает True если объект внутри коробки с бортиками
        """
        if team not in self.storage_zones:
            return False
            
        zone_center = self.storage_zones[team]
        half_size = self.cell_size / 2  # 125 мм
        
        # Проверяем попадание в квадрат 250x250 мм
        if (abs(object_position[0] - zone_center[0]) <= half_size and
            abs(object_position[1] - zone_center[1]) <= half_size):
            return True
        return False
