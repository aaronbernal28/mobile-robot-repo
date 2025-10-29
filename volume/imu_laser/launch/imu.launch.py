from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="imu_laser",
            executable="imu_calibrator",
            name="imu_calibrator",
            output="screen",
            parameters=[{"use_sim_time": True},
                        {"calibrate": 10}]
        ),
    ])
"""
ros2 launch imu_laser imu.launch.py

ros2 topic pub /robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, 
     angular: {x: 0.0, y: 0.0, z: -0.1}}"

"""