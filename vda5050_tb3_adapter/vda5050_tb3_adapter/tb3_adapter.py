#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright 2022 InOrbit, Inc.
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

# ROS dependencies
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import tf_transformations
import time

from vda5050_connector.utils import read_str_parameter

from vda5050_connector.action import NavigateToNode
from vda5050_connector.action import ProcessVDAAction
from nav2_msgs.action import NavigateToPose
from vda5050_connector.srv import GetState
from vda5050_connector.srv import SupportedActions
from nav_msgs.msg import Odometry

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from vda5050_msgs.msg import Node as VDANode
from vda5050_msgs.msg import AGVPosition as VDAAGVPosition
from vda5050_msgs.msg import Velocity as VDAVelocity
from vda5050_msgs.msg import OrderState as VDAOrderState
from vda5050_msgs.msg import CurrentAction as VDACurrentAction

NODE_NAME = "tb3_adapter"


class TB3Adapter(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.logger = self.get_logger()

        self._agv_position = VDAAGVPosition()
        self._velocity = VDAVelocity()
        self._driving = False

        self._active_node = VDANode()

        self.on_configure()
        self.logger.info("Node {} has started successfully.".format(NODE_NAME))

    def on_configure(self):
        self.read_parameters()

        base_interface_name = (
            f"{self.get_namespace()}/{self._manufacturer_name}/{self._robot_name}/"
        )

        self.get_adapter_state_srv = self.create_service(
            srv_type=GetState,
            srv_name=base_interface_name + self._get_state_svc_srv,
            callback=self.get_state_callback,
        )

        self.supported_actions_srv = self.create_service(
            SupportedActions,
            base_interface_name + self._supported_actions_svc_srv,
            lambda _: self.get_logger().info("Supported actions request"),
        )

        nav_to_node_action_client_cb_group = MutuallyExclusiveCallbackGroup()
        self._nav_to_node_ac_srv = ActionServer(
            node=self,
            action_type=NavigateToNode,
            action_name=base_interface_name + self._nav_to_node_act_srv,
            execute_callback=self.navigate_to_node_callback,
            callback_group=nav_to_node_action_client_cb_group,
        )

        self._process_vda_action_ac_srv = ActionServer(
            node=self,
            action_type=ProcessVDAAction,
            action_name=base_interface_name + self._vda_action_act_srv,
            execute_callback=self.process_vda_action_callback,
        )

        nav_to_pose_tb3_action_client_cb_group = MutuallyExclusiveCallbackGroup()
        self._nav_to_pose_tb3_action_client = ActionClient(
            node=self,
            action_type=NavigateToPose,
            action_name="/navigate_to_pose",
            callback_group=nav_to_pose_tb3_action_client_cb_group,
        )

        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

    def read_parameters(self):
        self._robot_name = read_str_parameter(self, "robot_name", "robot_1")
        self._manufacturer_name = read_str_parameter(
            self, "manufacturer_name", "robots"
        )
        self._serial_number = read_str_parameter(self, "serial_number", "robot_1")

        self._get_state_svc_srv = read_str_parameter(
            self, "get_state_svc_name", "adapter/get_state"
        )
        self._vda_action_act_srv = read_str_parameter(
            self, "vda_action_act_name", "adapter/vda_action"
        )
        self._nav_to_node_act_srv = read_str_parameter(
            self, "nav_to_node_act_name", "adapter/nav_to_node"
        )
        self._supported_actions_svc_srv = read_str_parameter(
            self, "supported_actions_svc_name", "adapter/supported_actions"
        )

    def odom_callback(self, odom_msg: Odometry):
        # Pose (map->base_link)
        to_frame = "map"
        from_frame = "base_link"
        self._agv_position.map_id = to_frame
        try:
            now = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(to_frame, from_frame, now)

            self._agv_position.x = trans.transform.translation.x
            self._agv_position.y = trans.transform.translation.y

            q = trans.transform.rotation
            _, _, theta = tf_transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
            self._agv_position.theta = theta
        except TransformException as ex:
            self.get_logger().info(
                "Could not transform {} to {}: {}".format(to_frame, from_frame, ex)
            )
            return

        # Twist
        self._velocity.vx = odom_msg.twist.twist.linear.x
        self._velocity.vy = odom_msg.twist.twist.linear.y
        self._velocity.omega = odom_msg.twist.twist.angular.z

    def get_state_callback(self, request, response):
        order_state = VDAOrderState()
        order_state.agv_position = self._agv_position
        order_state.velocity = self._velocity
        response.state = order_state
        return response

    def process_vda_action_callback(self, goal_handle):
        action = goal_handle.request.action
        result = ProcessVDAAction.Result()

        action_parameters = {}
        for action_parameter in action.action_parameters:
            action_parameters[action_parameter.key] = action_parameter.value

        self.logger.info(f"Parsed action parameters: {action_parameters}")

        if action.action_type == "initPosition":
            pose_with_cov_stamped = PoseWithCovarianceStamped()
            pose_with_cov_stamped.header.frame_id = action_parameters["mapId"]

            pose_with_cov_stamped.pose.pose.position.x = float(action_parameters["x"])
            pose_with_cov_stamped.pose.pose.position.y = float(action_parameters["y"])

            q = tf_transformations.quaternion_from_euler(
                0, 0, float(action_parameters["theta"])
            )
            pose_with_cov_stamped.pose.pose.orientation.x = q[0]
            pose_with_cov_stamped.pose.pose.orientation.y = q[1]
            pose_with_cov_stamped.pose.pose.orientation.z = q[2]
            pose_with_cov_stamped.pose.pose.orientation.w = q[3]

            self._init_pose_pub.publish(pose_with_cov_stamped)

        else:
            self.logger.info(
                f"Received unsupported action: '{action.action_type}'. "
                "Beep Boop Bop ... doing robot stuff"
            )

            current_action = VDACurrentAction(
                action_id=action.action_id,
                action_description=action.action_description,
                action_status=VDACurrentAction.INITIALIZING,
            )
            feedback_msg = ProcessVDAAction.Feedback(current_action=current_action)

            # Publish feedback with different actions states back to the
            # controller while waiting 5 seconds between feedback messages
            for action_status in [
                VDACurrentAction.INITIALIZING,
                VDACurrentAction.RUNNING,
                VDACurrentAction.PAUSED,
                VDACurrentAction.RUNNING,
                VDACurrentAction.FINISHED,
            ]:
                feedback_msg.current_action.action_status = action_status
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(5)

            result.result = feedback_msg.current_action
        goal_handle.succeed()

        return result

    def navigate_to_node_callback(self, goal_handle):
        node = goal_handle.request.node

        self.logger.info(
            f"Navigating to node '{goal_handle.request.node}', edge '{goal_handle.request.edge}'"
        )

        self._active_node = node
        goal_point = PoseStamped()

        goal_point.header.stamp = self.get_clock().now().to_msg()
        goal_point.header.frame_id = node.node_position.map_id

        goal_point.pose.position.x = node.node_position.x
        goal_point.pose.position.y = node.node_position.y

        q = tf_transformations.quaternion_from_euler(0, 0, node.node_position.theta)
        goal_point.pose.orientation.x = q[0]
        goal_point.pose.orientation.y = q[1]
        goal_point.pose.orientation.z = q[2]
        goal_point.pose.orientation.w = q[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_point

        self._nav_to_pose_tb3_action_client.wait_for_server()

        self.get_logger().info("Async goal for robot {} sent.".format("turtlebot_3"))

        self._nav_to_pose_tb3_send_goal_future = (
            self._nav_to_pose_tb3_action_client.send_goal_async(goal_msg)
        )
        self._driving = True

        self._nav_to_pose_tb3_send_goal_future.add_done_callback(
            self._nav_to_pose_tb3_goal_response_callback
        )

        while self._driving:
            pass

        goal_handle.succeed()

        result = NavigateToNode.Result()
        return result

    # Callbacks

    def _nav_to_pose_tb3_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")
        goal_handle.get_result_async().add_done_callback(
            self._nav_to_pose_tb3_get_result_callback
        )

    def _nav_to_pose_tb3_get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result: {}".format(result))
        self._driving = False
        self.get_logger().info(
            "The actions to do are: {}".format(self._active_node.actions)
        )


def main(args=None):
    rclpy.init(args=args)
    tb3_adapter_node = TB3Adapter()

    executor = MultiThreadedExecutor()
    executor.add_node(tb3_adapter_node)

    executor.spin()

    tb3_adapter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
