from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('description'), 'launch', 'construct.launch.py']
    )   
    
    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("robot_base"), "config", "ekf.yaml"]
    )
    
    # Добавляем запуск стратегии
    strategy_launch_path = PathJoinSubstitution(
        [FindPackageShare('strategy'), 'launch', 'strategy_launch.py']
    )
    
    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='odom_topic', 
            default_value='/odom',
            description='EKF out odometry topic'
        ),
        
        DeclareLaunchArgument(
            name='team',
            default_value='1',
            description='Team number (1 or 2)'
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path],
            remappings=[("odometry/filtered", LaunchConfiguration("odom_topic"))]
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(strategy_launch_path),
            launch_arguments={'team': LaunchConfiguration('team')}.items()
        ),
    ])
