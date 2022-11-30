# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    package_name = "vda5050_tb3_adapter"
    package_dir = get_package_share_directory(package_name)
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    nav_params_file = LaunchConfiguration('nav_params_file',)
    map = LaunchConfiguration('map')

    # Declare parameters

    declare_nav2 = DeclareLaunchArgument(
                        'nav_params_file',
                        default_value=os.path.join(package_dir, "config", 'carter_navigation_params.yaml'),
                        description='Full path to navigation param file to load')

    declare_map = DeclareLaunchArgument(
                        'map',
                        default_value=os.path.join(package_dir, 'maps', 'carter_warehouse_navigation.yaml'),
                        description='Full path to map file to load')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_launch_dir, "/bringup_launch.py"]),
        launch_arguments={
            "namespace": "",
            "use_namespace": "False",
            "use_composition": "False",
            "map": map,
            "use_sim_time": "False",
            "params_file": nav_params_file}.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_nav2)
    ld.add_action(declare_map)

    # Launch nodes
    ld.add_action(nav2_bringup_launch)
    return ld
