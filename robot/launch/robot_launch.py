from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from pathlib import Path
from launch_ros.actions import SetParameter
import os
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    robot_path = get_package_share_directory('robot')


    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(robot_path, 'config', 'config_file.rviz')]
    )
    
    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(robot_path, 
        'models', 'arm', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False},
        ]
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    
    camera1_pipeline = GroupAction(
            actions=[
            PushRosNamespace('camera1'),

            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                parameters=[robot_path+"/config/params_1.yaml"]
            ), 
           Node(
                package='apriltag_ros',
                executable='apriltag_node',
                remappings=[
                    ('image_rect', 'image_raw'),
                    ('camera_info', 'camera_info'),
                ],
                parameters=[robot_path+'/config/apriltags.yaml']
            )
        ]
    )
    
    camera2_pipeline = GroupAction(
        actions=[
            PushRosNamespace('camera2'),

            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                parameters=[robot_path+"/config/params_2.yaml"]
            ), 
           Node(
                package='apriltag_ros',
                executable='apriltag_node',
                remappings=[
                    ('image_rect', 'image_raw'),
                    ('camera_info', 'camera_info'),
                ],
                parameters=[robot_path+'/config/apriltags.yaml']
            )
        ]
    )

    tag_relay_node = Node(
        package='robot',
        executable='tag_relay'
    )


    joy = Node(
        package='joy',
        executable='joy_node'
    )

    joystick_control = Node(
        package='robot',
        executable='joystick'
    )

    arm_pos = Node(
        package='robot',
        executable='read_write_node'
    )

    
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(robot_path, 'config','ekf.yaml')]
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file':os.path.join(robot_path,'config/nav2_params.yaml')}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')),
        launch_arguments={'slam_params_file':os.path.join(robot_path,'config/mapper_params_localization.yaml')}.items(),
    )

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a2m8_launch.py' 
            )
        ),
        launch_arguments={'scan_mode': 'Standard'}.items()
    )

    #Controls motor and publishes imu and odom
    motor_control = Node(
        package='pass_nav_sig',
        executable='twist_to_arduino',
        name='motor_control',
        output='screen'
    )

    ros_bt = Node(
        package='ros_bt',
        executable='exec',
        name='exec_node',
        output='screen'
    )

    delayed_slam = TimerAction(
        period=5.0,  # <-- Set your delay in seconds here
        actions=[
            slam
        ]
    )

    delayed_nav2 = TimerAction(
        period=10.0,  # <-- Set your delay in seconds here
        actions=[
            nav2
        ]
    )


    delayed_exec = TimerAction(
        period=15.0,  # <-- Set your delay in seconds here
        actions=[
            ros_bt
        ]
    )


    return LaunchDescription([
        motor_control,
        ekf,
        rplidar,
        camera1_pipeline,
        camera2_pipeline,
        tag_relay_node,
        #joy,
        #joystick_control,
        arm_pos,
        robot_state_publisher,
        #joint_state_publisher_gui,
        delayed_slam,
        delayed_nav2,
        delayed_exec,
            ])
