# BSD 3-Clause License
#
# Copyright 2022 Clearpath Robotics, Inc.
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

# Import dependencies
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Get the launch directory
    package_name = "vda5050_template_package"
    package_dir = get_package_share_directory(package_name)

    package_name_connector = "vda5050_connector"
    package_dir_connector = get_package_share_directory(package_name_connector)

    namespace = LaunchConfiguration("namespace")
    parameters_config_file = LaunchConfiguration("parameters_config_file")

    # Declare parameters

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="vda5050_connector",
        description="Namespace to use",
    )

    declare_parameters_config_file_cmd = DeclareLaunchArgument(
        "parameters_config_file",
        default_value=os.path.join(package_dir, "config", "connector.yaml"),
        description="Full path to the parameters config file to use",
    )

    # Nodes

    mqtt_bridge_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir_connector, "launch", "mqtt_bridge.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "parameters_config_file": parameters_config_file,
        }.items(),
    )

    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir_connector, "launch", "controller.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "parameters_config_file": parameters_config_file,
        }.items(),
    )

    adapter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, "launch", "adapter_node.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "parameters_config_file": parameters_config_file,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace)
    ld.add_action(declare_parameters_config_file_cmd)

    # Launch nodes
    ld.add_action(mqtt_bridge_node)
    ld.add_action(controller_node)
    ld.add_action(adapter_node)

    return ld
