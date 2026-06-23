from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # PX4_2 的 ROS 2 话题前缀。
    px4_namespace_arg = DeclareLaunchArgument(
        'px4_namespace',
        default_value='/px4_2/fmu/',
        description='Target UAV PX4 topic namespace.',
    )
    # 低层 offboard 执行器订阅的速度指令话题。
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/target_uav/cmd_vel_body',
        description='Body-frame velocity command topic for the target UAV.',
    )
    # 目标机起飞高度。
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='3.0',
        description='Takeoff altitude in meters.',
    )
    # 目标机起飞朝向。
    takeoff_yaw_arg = DeclareLaunchArgument(
        'takeoff_yaw',
        default_value='1.57',
        description='Takeoff yaw in radians.',
    )
    # 低层 offboard 控制循环频率。
    control_rate_hz_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='10.0',
        description='Control loop frequency.',
    )
    # 外部速度指令停止发布后，低层执行器保留最后动作的时间。
    cmd_timeout_sec_arg = DeclareLaunchArgument(
        'cmd_timeout_sec',
        default_value='0.5',
        description='How long to keep the last velocity command before hovering.',
    )
    node = Node(
        package='target_uav_offboard',
        executable='target_offboard_control',
        output='screen',
        parameters=[{
            'px4_namespace': LaunchConfiguration('px4_namespace'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'takeoff_height': LaunchConfiguration('takeoff_height'),
            'takeoff_yaw': LaunchConfiguration('takeoff_yaw'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'cmd_timeout_sec': LaunchConfiguration('cmd_timeout_sec'),
        }],
    )
    return LaunchDescription([
        px4_namespace_arg,
        cmd_vel_topic_arg,
        takeoff_height_arg,
        takeoff_yaw_arg,
        control_rate_hz_arg,
        cmd_timeout_sec_arg,
        node,
    ])
