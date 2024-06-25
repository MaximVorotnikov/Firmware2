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
            'gz_args': '-r third_scene_world_v1.sdf'
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

    args0 = {
        'fcu_url': 'udp://:14540@localhost:14580',
        'tgt_system' : '1',
        }.items()     

    launch_action = GroupAction([
        PushRosNamespace('uav1'),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([os.path.join(
            get_package_share_directory('mavros'), 'launch/'),
            '/px4.launch']), launch_arguments=args0
        ),
    ])

    ld.add_action(launch_action)


    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[ 
                    '/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/depth_cam_drone/base_link/sensor/depth_cam/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/lidar/base_link/sensor/laser/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    ],
         parameters=[{'qos_overrides./uav1.subscriber.reliability': 'reliable',}],
                         
        remappings=[
                    ('/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/image', '/uav1/camera'),
                    ('/world/default/model/uav1/link/lidar/base_link/sensor/laser/scan', '/uav1/scan'),
                    ('/world/default/model/uav1/link/depth_cam_drone/base_link/sensor/depth_cam/depth_image', '/uav1/depth_camera'),
                    ('/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/image', '/uav1/camera_down')],
        output='screen'
        )
    
    ld.add_action(bridge)

    return ld