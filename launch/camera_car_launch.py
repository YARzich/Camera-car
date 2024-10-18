import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('camera_car_simple'),
        'description',
        'car.urdf.xacro')
    robot_description = {'robot_description': xacro.process_file(urdf_path).toxml()}

    rviz = os.path.join(
        get_package_share_directory('camera_car_simple'),
        'rviz',
        'camera_car.rviz')

    sim_time = LaunchConfiguration(
        'use_sim_time',
        default='true')

    # State publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            sim_time,
        ],
    )

    # RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz]
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': 'empty.sdf', "use_sim_time": 'True'}.items()
    )
    spawn_entity = Node(package='ros_gz_sim',
                        executable='create',
                        output='screen',
                        arguments=['-topic', '/robot_description',
                                   '-name', 'camera_car',
                                   '-z', '1'])

    # Controller Manager
    diff_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_controller'],
    )
    position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Teleoperation
    controller_converter = Node(
        package='camera_car_simple',
        executable='controller_converter',
        name='controller_converter',
        output='both',
    )

    teleoperation_keyboard = Node(
        package='teleoperation_dual_control',
        executable='teleoperation_dual_control_keyboard',
        name='teleoperation_keyboard',
        output='screen',
    )

    #Camera
    camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_info_bridge',
        arguments=[
            '/camera1/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    camera_image = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='image_bridge',
        arguments=[
            '/camera1/image@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz2,
        gazebo,
        spawn_entity,
        diff_controller,
        position_controller,
        joint_state_broadcaster,
        controller_converter,
        teleoperation_keyboard,
        camera_info,
        camera_image
    ])