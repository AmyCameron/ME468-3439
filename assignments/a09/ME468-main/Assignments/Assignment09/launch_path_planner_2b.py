import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_stack',
            namespace='',
            executable='simulation',
            name='simulation',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {"visualize":True},
                {"save":True},
                {"save_name":"part2b_ow"},
                {"initial_location":[-120,0,1.6]},
                {"initial_orientation":0.0},
                {"object_location_file":"data/object_detection/object_locations_2.csv"},
                {"random_object_count": 0},
                {"publish_state_directly":True},
                {"max_duration": 60.0},
                {"target_location":[120.0,0.0,0.0]}
            ],
        ),
        Node(
            package='control_stack',
            namespace='',
            executable='control_path_follower',
            name='control_path_follower',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {"use_sim_time": True},
                {"visualize":False},
                {"steering_kp": .2},
                {"steering_kd": 0.0},
                {"steering_ki": 0.0},
                {"lookahead": 10.0},
                {"speed_kp": 0.2},
                {"speed_kd": 0.0},
                {"speed_ki": 0.0},
                {"target_speed": 5.0}
            ],
        ),
        Node(
            package='control_stack',
            namespace='',
            executable='path_planner',
            name='path_planner',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {"use_sim_time": True},
                {"visualize":True},
                {"max_goal_weight":10.0},
                {"obstacle_weight":10.0},
                {"obstacle_radius":2.0},
                {"obstacle_effect_radius":10.0},
                {"target":[120.0,0.0]},
            ]
        ),
        Node(
            package='control_stack',
            namespace='',
            on_exit=launch.actions.Shutdown(),
            executable='object_detection_from_file',
            name='object_detection_from_file',
            parameters=[
                {"use_sim_time": True},
                {"visualize": False},
                {"object_file": "data/object_detection/object_locations_2.csv"},
                {"goal_location": [0,120]},
                {"field_of_view": 3.14},
                {"view_distance": 100.0}
            ]
        )
    ])
