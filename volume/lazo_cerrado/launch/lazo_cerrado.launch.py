from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import pi as M_PI

TIME_FACT = 0.1
FLOWER = [
    0., 0., 0., 0.,
    10./TIME_FACT, 3., 0., M_PI/4.,
    20./TIME_FACT, 0., 0., -M_PI/4.,
    30./TIME_FACT, 0., 3., 3*M_PI/4.,
    40./TIME_FACT, 0., 0., -3*M_PI/4.,
    50./TIME_FACT, -3., 0., 5*M_PI/4.,
    60./TIME_FACT, 0., 0., -5*M_PI/4.,
    70./TIME_FACT, 0., -3., 7*M_PI/4.,
    80./TIME_FACT, 0., 0., -7*M_PI/4.,
    90./TIME_FACT, 0., 0., 0.
]

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="modelo_diferencial",
            executable="pioneer_odometry_node",
            name="pioneer_odometry",
            output="screen",
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package="lazo_cerrado",
            executable="trajectory_follower_cl",
            name="trajectory_follower_cl",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {"goal_selection": "TIME_BASED"}, #FIXED_GOAL, TIME_BASED, PURSUIT_BASED
                {"fixed_goal_x": float(-2.0)},
                {"fixed_goal_y": float(1.0)},
                {"fixed_goal_a": float(-0.570796327*1.2)}#-0.785)}, # -1/2 * PI -0.570796327
            ],
        ),

        Node(
            package="lazo_abierto",
            executable="trajectory_generator",
            name="trajectory_generator",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {'trajectory_type': 'spline'}, #sin or spline
                {"stepping": float(0.1)},
                {"total_time": float(5.0)},
                {"amplitude": float(1.0)},
                {"cycles": float(1.0)},
                {'spline_waypoints': FLOWER}
            ],  
        ),
	# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)
        Node(
            package="lazo_cerrado",
            executable="logger",
            name="logger",
            output="screen",
            parameters=[{'use_sim_time': True}]
        ),
    ])