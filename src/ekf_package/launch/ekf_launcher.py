import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[os.path.join(
                os.path.dirname(__file__),
                'config',
                'navsat.yaml'
            )],
            remappings=[
                ('odometry/gps', '/earth/odometry/gps'),
                ('odometry/filtered', '/odom_imu'),
                ('gps/fix', '/earth/gps/fix')
            ]
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(
                os.path.dirname(__file__),
                'config',
                'ekf.yaml'
            )]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_base_to_gps',
            output='screen',
            arguments=['0', '0', '0.4', '0', '0', '0', 'base_link', 'gps_link']
        )

    ])
