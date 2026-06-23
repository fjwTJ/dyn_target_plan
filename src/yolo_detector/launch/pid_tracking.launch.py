from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/realsense/rgbd/image',
        description='RGB image topic.',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/realsense/rgbd/depth_image',
        description='Raw depth image topic.',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/realsense/rgbd/camera_info',
        description='Camera info topic.',
    )
    depth_16uc1_topic_arg = DeclareLaunchArgument(
        'depth_16uc1_topic',
        default_value='/realsense/rgbd/depth_image_16uc1',
        description='Converted 16UC1 depth image topic.',
    )
    target_topic_arg = DeclareLaunchArgument(
        'target_topic',
        default_value='/perception/target_xyz',
        description='Target point topic from depth projection.',
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/uav/cmd_vel_body',
        description='Body-frame velocity command topic for offboard control.',
    )
    target_lost_timeout_sec_arg = DeclareLaunchArgument(
        'target_lost_timeout_sec',
        default_value='1.5',
        description='Target lost timeout in seconds.',
    )
    use_sim_tf_arg = DeclareLaunchArgument(
        'use_sim_tf',
        default_value='true',
        description='Use simulation camera extrinsics in camera_tf_publisher_node.',
    )
    run_yolo_arg = DeclareLaunchArgument(
        'run_yolo',
        default_value='true',
        description='Run YOLO detector.',
    )
    run_target_depth_arg = DeclareLaunchArgument(
        'run_target_depth',
        default_value='true',
        description='Run target depth projection node.',
    )
    run_tracker_arg = DeclareLaunchArgument(
        'run_tracker',
        default_value='true',
        description='Run PID target tracker.',
    )
    run_target_lost_monitor_arg = DeclareLaunchArgument(
        'run_target_lost_monitor',
        default_value='true',
        description='Run target lost monitor.',
    )
    yolo_enable_visualization_arg = DeclareLaunchArgument(
        'yolo_enable_visualization',
        default_value='true',
        description='Enable YOLO OpenCV visualization window.',
    )

    depth_convert = Node(
        package='depth_convert',
        executable='depth_convert_node',
        parameters=[{
            'input_topic': LaunchConfiguration('depth_topic'),
            'output_topic': LaunchConfiguration('depth_16uc1_topic'),
        }],
        output='screen',
    )
    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        condition=IfCondition(LaunchConfiguration('run_yolo')),
        parameters=[{
            'image_topic': LaunchConfiguration('rgb_topic'),
            'enable_visualization': LaunchConfiguration('yolo_enable_visualization'),
        }],
        output='screen',
    )
    target_depth_node = Node(
        package='yolo_detector',
        executable='target_depth_node',
        condition=IfCondition(LaunchConfiguration('run_target_depth')),
        parameters=[{
            'camera_info_topic': LaunchConfiguration('camera_info_topic'),
            'depth_topic': LaunchConfiguration('depth_16uc1_topic'),
            'target_topic': LaunchConfiguration('target_topic'),
            'depth_value_type': 'ray_range',
        }],
        output='screen',
    )
    camera_tf_publisher_node = Node(
        package='yolo_detector',
        executable='camera_tf_publisher_node',
        parameters=[{'use_sim_tf': LaunchConfiguration('use_sim_tf')}],
        output='screen',
    )
    target_lost_monitor_node = Node(
        package='yolo_detector',
        executable='target_lost_monitor_node',
        condition=IfCondition(LaunchConfiguration('run_target_lost_monitor')),
        parameters=[{
            'target_topic': LaunchConfiguration('target_topic'),
            'target_timeout_sec': LaunchConfiguration('target_lost_timeout_sec'),
        }],
        output='screen',
    )
    tracker_node = Node(
        package='yolo_detector',
        executable='tracker_node',
        condition=IfCondition(LaunchConfiguration('run_tracker')),
        parameters=[{
            'target_topic': LaunchConfiguration('target_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
        }],
        output='screen',
    )

    return LaunchDescription([
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        depth_16uc1_topic_arg,
        target_topic_arg,
        cmd_vel_topic_arg,
        target_lost_timeout_sec_arg,
        use_sim_tf_arg,
        run_yolo_arg,
        run_target_depth_arg,
        run_tracker_arg,
        run_target_lost_monitor_arg,
        yolo_enable_visualization_arg,
        depth_convert,
        yolo_node,
        target_depth_node,
        camera_tf_publisher_node,
        target_lost_monitor_node,
        tracker_node,
    ])
