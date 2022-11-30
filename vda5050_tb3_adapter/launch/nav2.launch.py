# BSD 3-Clause License
#
# Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the InOrbit, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

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
