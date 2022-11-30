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

import argparse
import sys
import carb
from omni.isaac.kit import SimulationApp

ENV_USD_PATH = "/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd"

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--scenario_model", "--scenario-model",
                    default="http://ov-isaac-dev.nvidia.com/omniverse://ov-isaac-dev.nvidia.com/NVIDIA/Assets/Isaac/2022.1/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd")
args, _ = parser.parse_known_args()

CONFIG = {"renderer": "RayTracedLighting", "headless": args.headless}

# Example ROS2 bridge sample demonstrating the manual loading of Multiple Robot Navigation scenario
simulation_app = SimulationApp(CONFIG)
import omni
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils import viewports, stage, extensions, prims, rotations, nucleus
from pxr import Sdf

from omni.isaac.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

# Locate assets root folder to load sample
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

usd_path = args.scenario_model
omni.usd.get_context().open_stage(usd_path, None)

# Wait two frames so that stage starts loading
simulation_app.update()
simulation_app.update()

print("Loading stage...")
from omni.isaac.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

simulation_context = SimulationContext(stage_units_in_meters=1.0)

simulation_app.update()

simulation_context.play()

simulation_app.update()

while simulation_app.is_running():

    # runs with a realtime clock
    simulation_app.update()

simulation_context.stop()
simulation_app.close()
