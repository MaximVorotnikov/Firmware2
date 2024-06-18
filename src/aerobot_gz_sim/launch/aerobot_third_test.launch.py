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
            'gz_args': '-r third_scene_world_v2.sdf'
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

    # spawn_agressivniy_drone2 = ExecuteProcess(
    #     cmd=[[
    #         'PX4_SYS_AUTOSTART=4012 PX4_GZ_MODEL_NAME=uav2 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 1',
    #     ]],
    #     shell=True
    # )
    
    # ld.add_action(spawn_agressivniy_drone2)

    # spawn_agressivniy_drone3 = ExecuteProcess(
    #     cmd=[[
    #         'PX4_SYS_AUTOSTART=4012 PX4_GZ_MODEL_NAME=uav3 ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 -i 2',
    #     ]],
    #     shell=True
    # )
    
    # ld.add_action(spawn_agressivniy_drone3)


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

    # args1 = {
    #     'fcu_url': 'udp://:14541@localhost:14581',
    #     'tgt_system' : '2',
    #     }.items()     

    # launch_action = GroupAction([
    #     PushRosNamespace('uav2'),
    #     IncludeLaunchDescription(
    #         XMLLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('mavros'), 'launch/'),
    #         '/px4.launch']), launch_arguments=args1
    #     ),
    # ])

    # ld.add_action(launch_action)

    # args2 = {
    #     'fcu_url': 'udp://:14542@localhost:14582',
    #     'tgt_system' : '3',
    #     }.items()     

    # launch_action = GroupAction([
    #     PushRosNamespace('uav3'),
    #     IncludeLaunchDescription(
    #         XMLLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('mavros'), 'launch/'),
    #         '/px4.launch']), launch_arguments=args2
    #     ),
    # ])

    # ld.add_action(launch_action)

    # args2 = {
    #     'fcu_url': 'udp://:14542@localhost:14582',
    #     'tgt_system' : '3',
    #     }.items()     

    # launch_action = GroupAction([
    #     PushRosNamespace('uav3'),
    #     IncludeLaunchDescription(
    #         XMLLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('mavros'), 'launch/'),
    #         '/px4.launch']), launch_arguments=args2
    #     ),
    # ])

    # ld.add_action(launch_action)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[ 
                    # '/r0/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    # '/r0/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    # '/uav1/mono_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/depth_cam_drone/base_link/sensor/depth_cam/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/lidar/base_link/sensor/laser/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    # '/world/default/model/uav2/link/mono_cam/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/world/default/model/uav3/link/mono_cam/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/uav3/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/r0/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/r0/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    # '/r1/rgbd_camera/image@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
                    # '/uav1/mono_camera_down/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/world/default/model/uav2/link/mono_cam_down/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/world/default/model/uav3/link/mono_cam_down/base_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/uav3/camera_down@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/r1/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
                    # '/r1/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                    # '/model/r1/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    # '/model/r1/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    # '/r1/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
                    ],
         parameters=[{'qos_overrides./uav1.subscriber.reliability': 'reliable',}],
                    #   'qos_overrides./uav2.subscriber.reliability': 'reliable',
                        # 'qos_overrides./uav2.subscriber.reliability': 'reliable',
                        # 'qos_overrides./uav3.subscriber.reliability': 'reliable'}],
                         
        remappings=[
                    ('/world/default/model/uav1/link/mono_cam/base_link/sensor/imager/image', '/uav1/camera'),
                    ('/world/default/model/uav1/link/lidar/base_link/sensor/laser/scan', '/uav1/scan'),
                    ('/world/default/model/uav1/link/depth_cam_drone/base_link/sensor/depth_cam/depth_image', '/uav1/depth_camera'),
                    # ('/world/default/model/uav2/link/mono_cam/base_link/sensor/imager/image', '/uav2/camera'),
                    # ('/world/default/model/uav3/link/mono_cam/base_link/sensor/imager/image', '/uav3/camera'),
                    ('/world/default/model/uav1/link/mono_cam_down/base_link/sensor/imager/image', '/uav1/camera_down')],
                    # ('/world/default/model/uav2/link/mono_cam_down/base_link/sensor/imager/image', '/uav2/camera_down'),
                    # ('/world/default/model/uav3/link/mono_cam_down/base_link/sensor/imager/image', '/uav3/camera_down')],
        output='screen'
        )
    
    ld.add_action(bridge)

    return ld