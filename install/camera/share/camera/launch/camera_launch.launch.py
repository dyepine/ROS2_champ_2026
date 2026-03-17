from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():

    package_path = get_package_share_directory('camera')

    default_config_path = os.path.join(
        package_path, 'config', 'camera_calibration_config.yaml')
    
    # Добавляем аргумент для выбора команды
    team_arg = DeclareLaunchArgument(
        'team',
        default_value='1',
        description='Team number (1 or 2)'
    )

    bve_pose_node = Node(
        package='camera',
        executable='camera_bve_pose',
        parameters=[{'team': LaunchConfiguration('team')}]
    )
    
    image_raw_pub_node = Node(
        package='camera',
        executable='image_raw_publisher',
    )
    
    # Добавляем узел детекции объектов
    object_detector_node = Node(
        package='camera',
        executable='object_detector',
        parameters=[{'team': LaunchConfiguration('team')}]
    )

    ld = LaunchDescription()
    ld.add_action(team_arg)
    ld.add_action(bve_pose_node)
    ld.add_action(image_raw_pub_node)
    ld.add_action(object_detector_node)

    return ld
