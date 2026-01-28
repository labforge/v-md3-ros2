# coding: utf-8
"""
******************************************************************************
*  Copyright 2025 Labforge Inc.                                              *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License");            *
* you may not use this project except in compliance with the License.        *
* You may obtain a copy of the License at                                    *
*                                                                            *
*     https://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
******************************************************************************

ROS2 driver interface node that publishes dummy radar data for testing.

"""
__author__ = "Thomas Reidemeister <thomas@labforge.ca>"
__copyright__ = "Copyright 2025, Labforge Inc."

from launch_ros.actions import Node
from launch import LaunchDescription

# FIXME: Eventually parameterize this
SNOWFLEA_NAMESPACE = 'snowflea1'

def generate_launch_description():
    """
    Launch the VMD3 radar driver node.
    """
    return LaunchDescription([
        Node(
            package='vmd3_radar_driver',
            executable='driver',
            name='vmd3_radar_node',
            namespace=f'{SNOWFLEA_NAMESPACE}/vmd3',
            output='screen',

            parameters=[
                {'address': '192.168.1.201'},
                {'sensitivity': 15}, # Maximum sensitivity
                {'mode': 6},         # Mode 7 in datasheet (since API is zero indexed, p18)
                {'stationary' : True},
            ],
        ),
    ])
