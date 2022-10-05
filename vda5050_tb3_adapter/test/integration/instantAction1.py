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
from vda5050_msgs.msg import InstantActions
from vda5050_msgs.msg import Action
from vda5050_msgs.msg import ActionParameter

from paho.mqtt import client as mqtt_client
import ssl
import json
import os
from rosidl_runtime_py import message_to_ordereddict
from vda5050_connector.utils import json_snake_to_camel_case

MANUFACTURER = "OSRF"
SERIAL_NUMBER = "TB3_1"

instant_actions = InstantActions(
    header_id=0,
    timestamp=get_vda5050_ts(),
    version="1.1.1",
    manufacturer=MANUFACTURER,
    serial_number=SERIAL_NUMBER,
    actions=[
        Action(
            action_type="initPosition",
            action_id=str(uuid4()),
            action_description="",
            blocking_type=Action.NONE,
            action_parameters=[
                ActionParameter(key="x", value="2.0"),
                ActionParameter(key="y", value="-0.5"),
                ActionParameter(key="theta", value="0.0"),
                ActionParameter(key="mapId", value="map"),
            ],
        )
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

    instant_actions_json = json_snake_to_camel_case(
        json.dumps(message_to_ordereddict(instant_actions))
    )
    _mqtt_client.connect(mqtt_address, mqtt_port)
    _mqtt_client.publish(
        f"uagv/v1/{MANUFACTURER}/{SERIAL_NUMBER}/instantActions",
        json.dumps(instant_actions_json),
    )
    _mqtt_client.loop_start()
    print(f"Msg published: {instant_actions_json}")


if __name__ == "__main__":
    main()
