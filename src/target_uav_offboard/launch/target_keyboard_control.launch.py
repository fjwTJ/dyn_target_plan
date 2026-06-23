from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/target_uav/cmd_vel_body',
        description='Target UAV body-frame velocity command topic.',
    )
    publish_rate_hz_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='20.0',
        description='Velocity command publish rate.',
    )
    linear_step_arg = DeclareLaunchArgument(
        'linear_step',
        default_value='0.1',
        description='Increment for body-frame x/y velocity keys.',
    )
    vertical_step_arg = DeclareLaunchArgument(
        'vertical_step',
        default_value='0.05',
        description='Increment for body-frame z velocity keys.',
    )
    yaw_step_arg = DeclareLaunchArgument(
        'yaw_step',
        default_value='0.1',
        description='Increment for yaw-rate keys.',
    )
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='1.0',
        description='Maximum absolute body-frame x/y velocity.',
    )
    max_vertical_speed_arg = DeclareLaunchArgument(
        'max_vertical_speed',
        default_value='0.5',
        description='Maximum absolute body-frame z velocity.',
    )
    max_yaw_rate_arg = DeclareLaunchArgument(
        'max_yaw_rate',
        default_value='1.0',
        description='Maximum absolute yaw rate.',
    )

    node = Node(
        package='target_uav_offboard',
        executable='target_keyboard_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'linear_step': LaunchConfiguration('linear_step'),
            'vertical_step': LaunchConfiguration('vertical_step'),
            'yaw_step': LaunchConfiguration('yaw_step'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
            'max_vertical_speed': LaunchConfiguration('max_vertical_speed'),
            'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
        }],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        publish_rate_hz_arg,
        linear_step_arg,
        vertical_step_arg,
        yaw_step_arg,
        max_linear_speed_arg,
        max_vertical_speed_arg,
        max_yaw_rate_arg,
        node,
    ])
