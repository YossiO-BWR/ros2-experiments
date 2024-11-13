# Copyright 2021 The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.event_handlers.on_shutdown import OnShutdown
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from lifecycle_msgs.msg import Transition

import os
import pathlib

from ament_index_python import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def get_pkg_path(pkg_name, *args):
    try:
        return str(pathlib.Path(get_package_share_directory(pkg_name), *args))
    except (PackageNotFoundError, LookupError) as e:
        pass
    except Exception as e:
        print(f"Exception: {e}")
    return ""


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="yo_exp2",
                executable="comp_pub_node",
                name="talker",
                parameters=[
                    {
                        "pubs_count": 10,
                        "queue_size": 10,
                        "hz": 10.0,
                        "data_size": 31457280,
                        "best_effort": False,
                    }
                ],
                emulate_tty=True,
                output="screen",
            ),
            Node(
                package="yo_exp2",
                executable="comp_sub_node",
                name="listener",
                parameters=[
                    {
                        "pubs_count": 2,
                        "queue_size": 1,
                        "best_effort": False,
                    }
                ],
                emulate_tty=True,
                output="screen",
            ),
        ],
    )
