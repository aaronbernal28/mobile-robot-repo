from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import pi as M_PI
#import sys
#from pathlib import Path
#sys.path.insert(0, str(Path(__file__).parent))
#
#from spline_waypoints import generate_random_waypoints 
## pip install ortools
## pip install numpy >= 1.17.3

M_PI = float(M_PI)

CUADRADO = [0., 0., 0., 0.,
            15., 5., 0., M_PI / 2.,
            30., 5., 5., M_PI,
            45., 0., 5., 3*M_PI / 2.,
            60., 0., 0., 0.]

#RANDOM_WAYPOINTS = generate_random_waypoints(
#    range=3, 
#    samples=6, 
#    alpha_time=2.5, 
#    seed=28)
FLOWER = [
    0., 0., 0., 0.,
    10., 3., 0., M_PI/4.,
    20., 0., 0., -M_PI/4.,
    30., 0., 3., 3*M_PI/4.,
    40., 0., 0., -3*M_PI/4.,
    50., -3., 0., 5*M_PI/4.,
    60., 0., 0., -5*M_PI/4.,
    70., 0., -3., 7*M_PI/4.,
    80., 0., 0., -7*M_PI/4.,
    90., 0., 0., 0.
]


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
                {'amplitude': 1.0},
                {'cycles': 1.0},
                {'spline_waypoints': FLOWER} # hay que hacer colcon build --packages-select lazo_abierto para guardar los cambios
            ]
        )
    ])
# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)