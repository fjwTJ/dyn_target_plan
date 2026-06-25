from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    px4_dir_arg = DeclareLaunchArgument(
        'px4_dir',
        default_value=[EnvironmentVariable('HOME'), '/PX4-Autopilot'],
        description='PX4-Autopilot source directory.',
    )
    run_main_px4_arg = DeclareLaunchArgument(
        'run_main_px4',
        default_value='true',
        description='Start the main PX4 SITL with the gz_x500_depth model.',
    )
    run_target_px4_arg = DeclareLaunchArgument(
        'run_target_px4',
        default_value='true',
        description='Start the target PX4 SITL instance.',
    )
    target_px4_instance_arg = DeclareLaunchArgument(
        'target_px4_instance',
        default_value='2',
        description='Target PX4 instance id.',
    )
    target_px4_sys_autostart_arg = DeclareLaunchArgument(
        'target_px4_sys_autostart',
        default_value='4001',
        description='Target PX4_SYS_AUTOSTART value.',
    )
    target_px4_model_arg = DeclareLaunchArgument(
        'target_px4_model',
        default_value='gz_x500',
        description='Target PX4 simulation model.',
    )
    target_px4_model_pose_arg = DeclareLaunchArgument(
        'target_px4_model_pose',
        default_value='5,2',
        description='Target model spawn pose, such as x,y or x,y,z,...',
    )
    target_spawn_delay_sec_arg = DeclareLaunchArgument(
        'target_spawn_delay_sec',
        default_value='8.0',
        description='Delay before starting the target PX4 instance.',
    )
    run_xrce_agent_arg = DeclareLaunchArgument(
        'run_xrce_agent',
        default_value='true',
        description='Start Micro XRCE-DDS Agent.',
    )
    xrce_port_arg = DeclareLaunchArgument(
        'xrce_port',
        default_value='8888',
        description='Micro XRCE-DDS Agent UDP port.',
    )
    run_camera_bridges_arg = DeclareLaunchArgument(
        'run_camera_bridges',
        default_value='true',
        description='Bridge RGB, depth, and camera info topics from Gazebo to ROS 2.',
    )
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/realsense/rgbd/image',
        description='RGB image topic in Gazebo and ROS 2.',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/realsense/rgbd/depth_image',
        description='Depth image topic in Gazebo and ROS 2.',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/realsense/rgbd/camera_info',
        description='Camera info topic in Gazebo and ROS 2.',
    )

    main_px4 = ExecuteProcess(
        cmd=['env', 'make', 'px4_sitl_default', 'gz_x500_depth'],
        cwd=LaunchConfiguration('px4_dir'),
        condition=IfCondition(LaunchConfiguration('run_main_px4')),
        output='screen',
        emulate_tty=True,
    )
    target_px4 = ExecuteProcess(
        cmd=[
            'env',
            'PX4_GZ_STANDALONE=1',
            PythonExpression([
                "'PX4_SYS_AUTOSTART=' + '",
                LaunchConfiguration('target_px4_sys_autostart'),
                "'",
            ]),
            PythonExpression([
                "'PX4_GZ_MODEL_POSE=' + '",
                LaunchConfiguration('target_px4_model_pose'),
                "'",
            ]),
            PythonExpression([
                "'PX4_SIM_MODEL=' + '",
                LaunchConfiguration('target_px4_model'),
                "'",
            ]),
            './build/px4_sitl_default/bin/px4',
            '-i',
            LaunchConfiguration('target_px4_instance'),
        ],
        cwd=LaunchConfiguration('px4_dir'),
        condition=IfCondition(
            PythonExpression([
                "'",
                LaunchConfiguration('run_main_px4'),
                "' == 'true' and '",
                LaunchConfiguration('run_target_px4'),
                "' == 'true'",
            ])
        ),
        output='screen',
        emulate_tty=True,
    )
    delayed_target_px4 = TimerAction(
        period=LaunchConfiguration('target_spawn_delay_sec'),
        actions=[target_px4],
    )
    xrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', LaunchConfiguration('xrce_port')],
        condition=IfCondition(LaunchConfiguration('run_xrce_agent')),
        output='screen',
        emulate_tty=True,
    )

    bridge_rgb = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=IfCondition(LaunchConfiguration('run_camera_bridges')),
        arguments=[
            PythonExpression([
                "'",
                LaunchConfiguration('rgb_topic'),
                "@sensor_msgs/msg/Image@gz.msgs.Image'",
            ])
        ],
        output='screen',
    )
    bridge_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=IfCondition(LaunchConfiguration('run_camera_bridges')),
        arguments=[
            PythonExpression([
                "'",
                LaunchConfiguration('depth_topic'),
                "@sensor_msgs/msg/Image@gz.msgs.Image'",
            ])
        ],
        output='screen',
    )
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=IfCondition(LaunchConfiguration('run_camera_bridges')),
        arguments=[
            PythonExpression([
                "'",
                LaunchConfiguration('camera_info_topic'),
                "@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'",
            ])
        ],
        output='screen',
    )

    return LaunchDescription([
        px4_dir_arg,
        run_main_px4_arg,
        run_target_px4_arg,
        target_px4_instance_arg,
        target_px4_sys_autostart_arg,
        target_px4_model_arg,
        target_px4_model_pose_arg,
        target_spawn_delay_sec_arg,
        run_xrce_agent_arg,
        xrce_port_arg,
        run_camera_bridges_arg,
        rgb_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        main_px4,
        delayed_target_px4,
        xrce_agent,
        bridge_rgb,
        bridge_depth,
        bridge_camera_info,
    ])
