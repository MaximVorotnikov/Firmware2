import os

import launch_ros.actions as actions

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r second_scene_world_v1.sdf'
        }.items(),
    )

    ld.add_action(gz_sim)

    spawn_agressivniy_drone1 = ExecuteProcess(
        cmd=[[
            'PX4_SYS_AUTOSTART=4013 PX4_GZ_MODEL_NAME=uav1 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 0',
        ]],
        shell=True
    )

    ld.add_action(spawn_agressivniy_drone1)


    laser2base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        exec_name="static_transform_laser_to_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "uav1/lidar/base_link/laser"]
    )

    ld.add_action(laser2base_link)

    depth2base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        exec_name="static_transform_depth_to_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "uav1/depth_cam_drone/base_link/depth_cam"]
    )

    ld.add_action(depth2base_link)

    depth2base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        exec_name="static_transform_odom_to_map",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    ld.add_action(depth2base_link)

    odom2base_link = Node(
        package="drone_dynamic_tf_broadcaster",
        executable="broadcaster",
        exec_name="drone_dynamic_tf_broadcaster",
        namespace="uav1"
    )

    ld.add_action(odom2base_link)


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[ 
                    '/model/vehicle_qr2/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/vehicle_qr3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/vehicle_qr4/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/vehicle_qr5/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/vehicle_qr6/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/vehicle_qr7/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/vehicle_qr8/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/model/vehicle_qr9/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/depth_cam_drone/base_link/sensor/depth_cam/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/lidar/base_link/sensor/laser/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/depth_cam_drone/base_link/sensor/depth_cam/depth_image/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    ],
        # parameters=[{'qos_overrides./uav1.subscriber.reliability': 'reliable',}],   
        parameters=[{'qos_overrides./uav1.subscriber.reliability': 'reliable',
                        'qos_overrides./model.subscriber.reliability': 'reliable'}],   
        remappings=[
                    ('/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/image', '/uav1/camera'),
                    ('/model/vehicle_qr2/cmd_vel', '/vehicle_qr2/cmd_vel'),
                    ('/model/vehicle_qr3/cmd_vel', '/vehicle_qr3/cmd_vel'),
                    ('/model/vehicle_qr4/cmd_vel', '/vehicle_qr4/cmd_vel'),
                    ('/model/vehicle_qr5/cmd_vel', '/vehicle_qr5/cmd_vel'),
                    ('/model/vehicle_qr6/cmd_vel', '/vehicle_qr6/cmd_vel'),
                    ('/model/vehicle_qr7/cmd_vel', '/vehicle_qr7/cmd_vel'),
                    ('/model/vehicle_qr8/cmd_vel', '/vehicle_qr8/cmd_vel'),
                    ('/model/vehicle_qr9/cmd_vel', '/vehicle_qr9/cmd_vel'),
                    ('/world/default/model/uav1/link/depth_cam_drone/base_link/sensor/depth_cam/depth_image', '/uav1/depth_camera'),
                    ('/world/default/model/uav1/link/lidar/base_link/sensor/laser/scan', '/uav1/scan'),
                    ('/world/default/model/uav1/link/depth_cam_drone/base_link/sensor/depth_cam/depth_image/points', '/uav1/depth/cloud'),
                    ('/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/image', '/uav1/camera_down')],
        output='screen'
        )
    
    ld.add_action(bridge)

    return ld