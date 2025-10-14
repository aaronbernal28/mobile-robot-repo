from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import pi as M_PI
M_PI = float(M_PI)

CUADRADO = [0., 0., 0., 0.,
            15., 5., 0., M_PI / 2.,
            30., 5., 5., M_PI,
            45., 0., 5., 3*M_PI / 2.,
            60., 0., 0., 0.]

def generate_launch_description():
    return LaunchDescription([
        # Use simulation time for all nodes
        Node(
            package='modelo_diferencial',
            executable='pioneer_odometry_node',
            name='pioneer_odometry',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'stepping': 0.1},
                {'trajectory_type': 'spline'}, # sin o spline
                {'total_time': 50.0},
                {'amplitude': 1},
                {'cycles': 1.0},
                {'spline_waypoints': CUADRADO} # hay que hacer colcon build --packages-select lazo_abierto para guardar los cambios
            ]
        )
    ])
# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)