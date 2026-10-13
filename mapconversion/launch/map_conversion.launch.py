from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        # The 2D maps carry the OctoMap's stamp and frame. A non-empty
        # map_frame replaces the frame only; the scene graph uses one frame
        # for every layer (scene_graph_ros: map_frame, default odom).
        DeclareLaunchArgument('map_frame', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='mapconversion',
            executable='map_conversion_oct_node',
            name='map_conversion',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'map_frame': LaunchConfiguration('map_frame'),
                'minimum_z': 1.0,
                'max_slope_ugv': 0.2,
                'slope_estimation_size': 2,
                'minimum_occupancy': 10,
                'partial_map_updates': True,
                #QoS parameters
                'subscriber_qos_reliable': True,
                'subscriber_qos_transient_local': False,
                'publisher_qos_reliable': True,
                # Latched: inc_dude and scene_graph_server subscribe
                # transient_local, which a volatile publisher cannot serve.
                'publisher_qos_transient_local': True,
            }],
            remappings=[
                ('octomap', 'octomap_binary')
            ]
        )
    ])
