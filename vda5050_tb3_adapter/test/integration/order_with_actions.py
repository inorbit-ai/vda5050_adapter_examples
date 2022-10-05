# BSD 3-Clause License
#
# Copyright (c) 2022 InOrbit, Inc.
# Copyright (c) 2022 Clearpath Robotics, Inc.
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

from uuid import uuid4
from vda5050_connector.utils import get_vda5050_ts
from vda5050_msgs.msg import Order
from vda5050_msgs.msg import Node
from vda5050_msgs.msg import NodePosition
from vda5050_msgs.msg import Action
from vda5050_msgs.msg import Edge

from paho.mqtt import client as mqtt_client
import ssl
import json
import os

from rosidl_runtime_py import message_to_ordereddict
from vda5050_connector.utils import json_snake_to_camel_case

MANUFACTURER = "OSRF"
SERIAL_NUMBER = "TB3_1"

order = Order(
    header_id=0,
    timestamp=get_vda5050_ts(),
    version="1.1.1",
    manufacturer=MANUFACTURER,
    serial_number=SERIAL_NUMBER,
    order_id=str(uuid4()),
    order_update_id=0,
    nodes=[
        Node(
            node_id="node1",
            sequence_id=0,
            released=True,
            node_position=NodePosition(
                x=2.0,
                y=0.95,
                theta=-0.66,
                allowed_deviation_x_y=0.0,
                allowed_deviation_theta=0.0,
                map_id="map",
            ),
            actions=[
                Action(
                    action_type="beep",
                    action_id=str(uuid4()),
                    action_description="Make a beep noise on node",
                    blocking_type="NONE",
                )
            ],
        ),
        Node(
            node_id="node2",
            sequence_id=2,
            released=True,
            node_position=NodePosition(
                x=1.18,
                y=-1.76,
                theta=0.0,
                allowed_deviation_x_y=0.0,
                allowed_deviation_theta=0.0,
                map_id="map",
            ),
            actions=[
                Action(
                    action_type="beep",
                    action_id=str(uuid4()),
                    action_description="Make a beep noise on node",
                    blocking_type="NONE",
                )
            ],
        ),
        Node(
            node_id="node3",
            sequence_id=4,
            released=True,
            node_position=NodePosition(
                x=-0.38,
                y=1.89,
                theta=0.0,
                allowed_deviation_x_y=0.0,
                allowed_deviation_theta=0.0,
                map_id="map",
            ),
            actions=[
                Action(
                    action_type="beep",
                    action_id=str(uuid4()),
                    action_description="Make a beep noise on node",
                    blocking_type="NONE",
                )
            ],
        ),
        Node(
            node_id="node4",
            sequence_id=6,
            released=True,
            node_position=NodePosition(
                x=-0.17,
                y=1.74,
                theta=-2.6,
                allowed_deviation_x_y=0.0,
                allowed_deviation_theta=0.0,
                map_id="map",
            ),
            actions=[
                Action(
                    action_type="beep",
                    action_id=str(uuid4()),
                    action_description="Make a beep noise on node",
                    blocking_type="NONE",
                )
            ],
        ),
        Node(
            node_id="node1",
            sequence_id=8,
            released=True,
            node_position=NodePosition(
                x=2.0,
                y=0.95,
                theta=-0.66,
                allowed_deviation_x_y=0.0,
                allowed_deviation_theta=0.0,
                map_id="map",
            ),
            actions=[
                Action(
                    action_type="beep",
                    action_id=str(uuid4()),
                    action_description="Make a beep noise on node",
                    blocking_type="NONE",
                )
            ],
        ),
    ],
    edges=[
        Edge(
            edge_id="edge1",
            sequence_id=1,
            released=True,
            start_node_id="node1",
            end_node_id="node2",
            max_speed=10.0,
            max_height=10.0,
            min_height=1.0,
        ),
        Edge(
            edge_id="edge2",
            sequence_id=3,
            released=True,
            start_node_id="node2",
            end_node_id="node3",
            max_speed=10.0,
            max_height=10.0,
            min_height=1.0,
        ),
        Edge(
            edge_id="edge3",
            sequence_id=5,
            released=True,
            start_node_id="node3",
            end_node_id="node4",
            max_speed=10.0,
            max_height=10.0,
            min_height=1.0,
        ),
        Edge(
            edge_id="edge4",
            sequence_id=7,
            released=True,
            start_node_id="node4",
            end_node_id="node1",
            max_speed=10.0,
            max_height=10.0,
            min_height=1.0,
        ),
    ],
)


def main():
    mqtt_address = os.environ.get("MQTT_ADDRESS", "localhost")
    mqtt_port = int(os.environ.get("MQTT_PORT", 1883))
    mqtt_username = os.environ.get("MQTT_USERNAME")
    mqtt_password = os.environ.get("MQTT_PASSWORD")

    _mqtt_client = mqtt_client.Client()

    if mqtt_username:
        _mqtt_client.tls_set(
            "/etc/ssl/certs/ca-certificates.crt", tls_version=ssl.PROTOCOL_TLSv1_2
        )
        _mqtt_client.username_pw_set(username=mqtt_username, password=mqtt_password)

    order_json = json_snake_to_camel_case(json.dumps(message_to_ordereddict(order)))
    _mqtt_client.connect(mqtt_address, mqtt_port)
    _mqtt_client.publish(
        f"uagv/v1/{MANUFACTURER}/{SERIAL_NUMBER}/order", json.dumps(order_json)
    )
    _mqtt_client.loop_start()
    print(f"Msg published: {order_json}")


if __name__ == "__main__":
    main()
