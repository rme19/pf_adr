#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def get_teleop_controller(context, *_, **kwargs) -> Node:
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]

    if controller == "joystick":
        node = Node(
            package="sjtu_drone_control",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )
    else:
        node = Node(
            package="sjtu_drone_control",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",
        )
    return [node]

def generate_launch_description():
    pf_adr_bringup_path = get_package_share_directory('pf_adr')

    yaml_file_path = os.path.join(
        get_package_share_directory('sjtu_drone_bringup'),
        'config', 'drone.yaml'
    )

    model_ns = "drone"

    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict.get("namespace", model_ns)

    # Número de beacons a lanzar
    num_beacons = 5
    

    # Lista de nodos de filtros de partículas, uno por beacon
    particle_filter_nodes = []
    for i in range(int(num_beacons)):
        particle_filter_nodes.append(
            Node(
                package='pf_adr',
                executable='particle_filter_node',
                name=f'particle_filter_node_{i}',
                parameters=[{
                    'total_num_particles': 5000,
                    'total_beacons': num_beacons,
                    'sigma': 0.1,
                    'noise_std': 0.1,
                    'radius': 2.5,
                    'beacon_id': i  # si el nodo usa este parámetro
                }],
                remappings=[
                    ("/distance_to_target", f"/beacon_{i}/distance_to_target"),
                    ("/pf/beacon_estimate", f"/pf/beacon_{i}/estimate"),
                    ("/pf/particles", f"/pf/beacon_{i}/particles"),
                ],
                output='screen',
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'num_beacons',
            default_value='5',
            description='Número total de balizas esperadas'
        ),
        DeclareLaunchArgument(
            "controller",
            default_value="keyboard",
            description="Type of controller: keyboard (default) or joystick",
        ),

        DeclareLaunchArgument(
            'fixed_frame',
            default_value='',
            description='If provided, sets the fixed frame in RViz.'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pf_adr_bringup_path, 'launch', 'sjtu_drone_fp.launch.py')
            )
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            namespace=model_ns,
            output='screen',
        ),
        Node(
            package='pf_adr',
            executable='beacon_activity_control',
            name='beacon_activity_control',
            parameters=[{
                'num_beacons': num_beacons,
                'timeout_sec': 1.0
            }]
        ),


        # Añadimos todos los nodos de filtros de partículas aquí
        *particle_filter_nodes,

        OpaqueFunction(
            function=get_teleop_controller,
            kwargs={'model_ns': model_ns},
        ),
    ])
