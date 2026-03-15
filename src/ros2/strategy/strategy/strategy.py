import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import UInt8, Int16, String
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from ament_index_python.packages import get_package_share_directory
import os

class Strategy(Node):
    def __init__(self):
        super().__init__('strategy_node')
        
        # Параметры поля из правил
        self.field_width = 1.75  # 1750 мм
        self.field_height = 1.25  # 1250 мм
        self.cell_size = 0.25     # 250 мм
        
        # Зоны начисления баллов (квадраты A1-A3 для команды 1, B1-B3 для команды 2)
        self.scoring_zones = {
            'team1': {
                'A1': np.array([-self.field_width/2 + self.cell_size*0.5, -self.field_height/2 + self.cell_size*0.5, 0.0]),
                'A2': np.array([0.0, -self.field_height/2 + self.cell_size*0.5, 0.0]),
                'A3': np.array([self.field_width/2 - self.field_size*1.5, -self.field_height/2 + self.cell_size*0.5, 0.0])
            },
            'team2': {
                'B1': np.array([-self.field_width/2 + self.cell_size*0.5, self.field_height/2 - self.cell_size*0.5, 0.0]),
                'B2': np.array([0.0, self.field_height/2 - self.cell_size*0.5, 0.0]),
                'B3': np.array([self.field_width/2 - self.cell_size*1.5, self.field_height/2 - self.cell_size*0.5, 0.0])
            }
        }
        
        # Зоны хранения (угловые коробки с бортиками)
        self.storage_zones = {
            'team1': np.array([-self.field_width/2 + self.cell_size/2, 
                               self.field_height/2 - self.cell_size/2, 0.0]),  # Левый верхний угол
            'team2': np.array([self.field_width/2 - self.cell_size/2, 
                              self.field_height/2 - self.cell_size/2, 0.0])   # Правый верхний угол
        }
        
        # Стартовые зоны
        self.start_zones = {
            'A': np.array([-self.field_width/2 + self.cell_size*1.5, 
                          -self.field_height/2 + self.cell_size*1.5, 0.0]),  # Команда 1
            'B': np.array([self.field_width/2 - self.cell_size*1.5, 
                          -self.field_height/2 + self.cell_size*1.5, 0.0])   # Команда 2
        }
        
        # Определяем команду из переменной окружения
        self.team = os.getenv("TEAM", "1")
        self.get_logger().info(f"Team: {self.team}")
        
        # Action Client для навигации
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # Publishers
        self.elevator_pub = self.create_publisher(UInt8, '/elevator/request', 10)
        self.screen_pub = self.create_publisher(Int16, '/screen', 10)
        self.obstacle_pub = self.create_publisher(String, '/obstacle_control', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/game_objects', 10)
        
        # Subscribers
        self.elevator_sub = self.create_subscription(UInt8, '/elevator/answer', self.elevator_callback, 10)
        self.timer_sub = self.create_subscription(UInt8, '/timer', self.timer_counter, 10)
        
        # Подписка на позиции объектов (нужно будет реализовать детекцию объектов)
        self.object_pose_sub = self.create_subscription(PoseStamped, '/object_poses', self.object_pose_callback, 10)
        
        # Bridge for CV
        self.bridge = CvBridge()
        
        # Timers
        self.create_timer(1.0, self.timer_callback)
        self.create_timer(3.0, self.screen_callback)
        
        # Загружаем параметры стратегии из YAML
        self.load_strategy_params()
        
        # Переменные состояния
        self.start_time = None
        self.match_duration = 90  # 90 секунд по правилам
        self.navigation_in_progress = False
        self.elevator_in_progress = False
        
        # Индексы для выполнения стратегии
        self.current_waypoint = 0
        self.current_elevator = 0
        self.current_obstacle = 0
        self.current_phase = 0  # 0: сбор объектов, 1: доставка в зоны, 2: финальная фаза
        
        # Счет
        self.score = 0
        self.objects_in_storage = []  # Список объектов в зоне хранения
        self.objects_in_zones = {}    # Словарь объектов по зонам
        
        # Детектированные объекты
        self.detected_objects = []  # Список [position, type, value]
        
    def load_strategy_params(self):
        """Загружает параметры стратегии из YAML файла"""
        try:
            # Пути к точкам в зависимости от команды
            package_path = get_package_share_directory('strategy')
            config_path = os.path.join(package_path, 'config', f'waypoints_team{self.team}.yaml')
            
            self.declare_parameter('waypoints')
            self.declare_parameter('elevator_order')
            self.declare_parameter('obstacle_release')
            self.declare_parameter('object_values')
            
            # Читаем параметры
            raw_waypoints = self.get_parameter("waypoints").get_parameter_value().double_array_value
            self.waypoints = []
            for i in range(0, len(raw_waypoints), 3):
                self.waypoints.append({
                    'x': raw_waypoints[i],
                    'y': raw_waypoints[i+1],
                    'yaw': raw_waypoints[i+2]
                })
                
            self.elevator_order = self.get_parameter("elevator_order").get_parameter_value().integer_array_value
            self.obstacle_release = self.get_parameter("obstacle_release").get_parameter_value().string_array_value
            self.object_values = self.get_parameter("object_values").get_parameter_value().integer_array_value
            
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load strategy params: {e}")
            # Загружаем пути по умолчанию
            self.set_default_waypoints()
    
    def set_default_waypoints(self):
        """Устанавливает пути по умолчанию на основе правил"""
        if self.team == "1":
            # Для команды 1: начинаем из зоны A
            start = self.start_zones['A']
            
            # Точки для сбора объектов
            self.waypoints = [
                {'x': start[0], 'y': start[1], 'yaw': 0.0},  # Старт
                {'x': 0.0, 'y': -0.4, 'yaw': 0.0},  # Центр нижней части
                {'x': 0.5, 'y': 0.0, 'yaw': 1.57},  # Правая часть
                {'x': -0.5, 'y': 0.0, 'yaw': -1.57},  # Левая часть
                {'x': -0.7, 'y': 0.5, 'yaw': 0.0},  # Зона хранения
            ]
        else:
            # Для команды 2: начинаем из зоны B
            start = self.start_zones['B']
            self.waypoints = [
                {'x': start[0], 'y': start[1], 'yaw': 3.14},
                {'x': 0.0, 'y': 0.4, 'yaw': 3.14},
                {'x': -0.5, 'y': 0.0, 'yaw': -1.57},
                {'x': 0.5, 'y': 0.0, 'yaw': 1.57},
                {'x': 0.7, 'y': -0.5, 'yaw': 3.14},
            ]
    
    def timer_callback(self):
        """Основной цикл стратегии"""
        if self.start_time is None:
            return
            
        elapsed_time = time.time() - self.start_time
        
        # Проверяем окончание матча
        if elapsed_time >= self.match_duration:
            self.finish_match()
            return
        
        # Логика стратегии в зависимости от времени
        if elapsed_time < 30:  # Первые 30 секунд - сбор объектов
            self.current_phase = 0
            self.collection_phase()
            
        elif elapsed_time < 60:  # Следующие 30 секунд - доставка в зоны
            self.current_phase = 1
            self.delivery_phase()
            
        else:  # Последние 30 секунд - финальная фаза (зона хранения)
            self.current_phase = 2
            self.storage_phase()
    
    def collection_phase(self):
        """Фаза сбора объектов"""
        if not self.navigation_in_progress and self.current_waypoint < 2:
            # Двигаемся к зонам с объектами
            self.navigate_to_waypoint(self.current_waypoint)
            
        # Если робот на месте и есть объекты для захвата
        if not self.navigation_in_progress and self.current_waypoint == 2:
            if not self.elevator_in_progress and self.current_elevator < len(self.elevator_order):
                # Захватываем объект
                self.elevator_publish(self.elevator_order[self.current_elevator])
    
    def delivery_phase(self):
        """Фаза доставки объектов в зоны начисления"""
        if not self.navigation_in_progress and self.current_waypoint < 4:
            # Двигаемся к зонам начисления
            self.navigate_to_waypoint(self.current_waypoint)
            
        # Выгружаем объекты в зонах
        if not self.navigation_in_progress and self.current_waypoint == 4:
            # Проверяем, в какой зоне находимся
            current_pos = self.get_current_position()
            zone = self.check_scoring_zone(current_pos)
            
            if zone and not self.elevator_in_progress:
                # Выгружаем объект в зону
                self.elevator_publish(3)  # Команда выгрузки
                
                # Начисляем баллы за объект в зоне
                if self.detected_objects:
                    obj = self.detected_objects.pop(0)
                    self.score += obj[2]  # Добавляем стоимость объекта
                    
                    # Дополнительные баллы за зону хранения
                    if self.check_storage_zone(current_pos):
                        self.score += 10  # Бонус за зону хранения
    
    def storage_phase(self):
        """Финальная фаза - доставка в зону хранения"""
        if not self.navigation_in_progress and self.current_waypoint < len(self.waypoints):
            # Двигаемся к зоне хранения
            self.navigate_to_waypoint(self.current_waypoint)
            
        # Если в зоне хранения, выгружаем все оставшиеся объекты
        if not self.navigation_in_progress and self.current_waypoint == len(self.waypoints) - 1:
            if not self.elevator_in_progress and self.detected_objects:
                self.elevator_publish(3)  # Выгрузка
                
                # Начисляем баллы (объекты в зоне хранения дают дополнительные баллы)
                obj = self.detected_objects.pop(0)
                self.score += obj[2] + 5  # Базовая стоимость + бонус
    
    def check_scoring_zone(self, position):
        """Проверяет, в какой зоне начисления находится позиция"""
        zones = self.scoring_zones[f'team{self.team}']
        for zone_name, zone_pos in zones.items():
            distance = np.linalg.norm(position[:2] - zone_pos[:2])
            if distance < self.cell_size / 2:
                return zone_name
        return None
    
    def check_storage_zone(self, position):
        """Проверяет, находится ли позиция в зоне хранения"""
        zone_pos = self.storage_zones[f'team{self.team}']
        distance = np.linalg.norm(position[:2] - zone_pos[:2])
        return distance < self.cell_size / 2
    
    def get_current_position(self):
        """Получает текущую позицию робота (нужно реализовать через odom)"""
        # TODO: Получать реальную позицию из odom
        return np.array([0.0, 0.0, 0.0])
    
    def object_pose_callback(self, msg):
        """Обрабатывает детектированные объекты на поле"""
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        
        # Определяем тип объекта по ID маркера
        # TODO: Реализовать определение типа объекта
        
        # Добавляем в список детектированных объектов
        self.detected_objects.append([position, 'unknown', 10])  # Временное значение
    
    def navigate_to_waypoint(self, waypoint_index):
        """Отправляет цель навигации"""
        if waypoint_index >= len(self.waypoints):
            return
            
        self.navigation_in_progress = True
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        wp = self.waypoints[waypoint_index]
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        
        # Конвертируем yaw в кватернион
        quat = R.from_euler('z', wp['yaw']).as_quat()
        goal_msg.pose.pose.orientation.x = quat[0]
        goal_msg.pose.pose.orientation.y = quat[1]
        goal_msg.pose.pose.orientation.z = quat[2]
        goal_msg.pose.pose.orientation.w = quat[3]
        
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.navigation_in_progress = False
            return
            
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        self.navigation_in_progress = False
        self.current_waypoint += 1
    
    def feedback_callback(self, feedback_msg):
        pass
    
    def elevator_publish(self, command):
        """Отправляет команду элеватору"""
        self.elevator_in_progress = True
        msg = UInt8()
        msg.data = command
        self.elevator_pub.publish(msg)
    
    def elevator_callback(self, msg):
        """Обрабатывает ответ от элеватора"""
        self.elevator_in_progress = False
        if self.current_elevator < len(self.elevator_order):
            self.current_elevator += 1
    
    def timer_counter(self, msg):
        """Синхронизация старта матча"""
        if self.start_time is None and msg.data == 1:
            self.start_time = time.time()
            self.get_logger().info("Match started!")
    
    def screen_callback(self):
        """Публикует текущий счет на экран"""
        msg = Int16()
        msg.data = self.score
        self.screen_pub.publish(msg)
        self.get_logger().info(f"Current score: {self.score}")
    
    def finish_match(self):
        """Завершение матча"""
        self.get_logger().info(f"Match finished! Final score: {self.score}")
        
        # Публикуем финальный счет
        msg = Int16()
        msg.data = self.score
        self.screen_pub.publish(msg)
        
        # Останавливаем робота
        self.stop_robot()
        
        # Завершаем узел
        rclpy.shutdown()
    
    def stop_robot(self):
        """Останавливает робота"""
        from geometry_msgs.msg import Twist
        stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        stop_msg = Twist()
        stop_pub.publish(stop_msg)
    
    def publish_objects_markers(self):
        """Публикует маркеры объектов для визуализации в RViz"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(self.detected_objects):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "objects"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = obj[0][0]
            marker.pose.position.y = obj[0][1]
            marker.pose.position.z = 0.0
            
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.06
            
            # Цвет в зависимости от типа объекта
            if obj[2] > 0:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = Strategy()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Strategy node stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
