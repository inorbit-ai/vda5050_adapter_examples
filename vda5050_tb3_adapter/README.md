# VDA5050 TB3 Adapter

ROS2 package with a Python implementation of the adapter for a [Turtlebot 3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/galactic-devel). It contains the following directories:

- **config**: provides a _connector_tb3.yaml_ file to configure the MQTT bridge, controller and adapter parameters.
- **launch**: provides two `.launch.py` files, one for running the `tb3_adapter` node alone and the other to run the whole connector (`mqtt_bridge` and `controller` nodes from the `vda5050_connector` package, as well as the `tb3_adapter` node).

## Running the TB3 adapter

To test the TB3 adapter sample, follow these steps:

- Execute the _simulation_ environment (see [docker environment](../docker/README.md)).

- Inside the _development_ container:

  ```sh
  # Build workspace packages
  colcon build && source install/setup.bash
  # Set gazebo simulated `tb3` initial pose
  ros2 topic pub -1 --qos-reliability reliable /initialpose geometry_msgs/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: -2.1, y: -0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0, w: 1.0000000}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}"
  # Launch the TB3 VDA5050 connector
  ros2 launch vda5050_tb3_adapter connector_tb3.launch.py
  ```

- Attach a new terminal to the _development_ container by running `docker compose exec development bash` inside the `docker/deployment/` folder (see [docker environment](../docker/README.md)). Send any of the following samples.

  <details>
  <summary>4 nodes order</summary>

  ```sh
    mosquitto_pub -h mosquitto -p 1883 -t uagv/v1/OSRF/TB3_1/order -m '
    {
        "orderId": "'$(cat /proc/sys/kernel/random/uuid)'",
        "orderUpdateId": 0,
        "version": "2.0.0",
        "manufacturer": "OSRF",
        "serialNumber": "TB3_1",
        "nodes": [
            {
                "nodeId": "node1",
                "released": true,
                "sequenceId": 0,
                "nodePosition": {
                    "x": 2.0,
                    "y": 0.95,
                    "theta": -0.66,
                    "mapId": "map"
                },
                "actions": []
            },
            {
                "nodeId": "node2",
                "released": true,
                "sequenceId": 2,
                "nodePosition": {
                    "x": 1.18,
                    "y": -1.76,
                    "theta": 0.0,
                    "mapId": "map"
                },
                "actions": []
            },
            {
                "nodeId": "node3",
                "released": true,
                "sequenceId": 4,
                "nodePosition": {
                    "x": -0.38,
                    "y": 1.89,
                    "theta": 0.0,
                    "mapId": "map"
                },
                "actions": []
            },
            {
                "nodeId": "node4",
                "released": true,
                "sequenceId": 6,
                "nodePosition": {
                    "x": -0.17,
                    "y": 1.74,
                    "theta": -2.6,
                    "mapId": "map"
                },
                "actions": []
            },
            {
                "nodeId": "node1",
                "released": true,
                "sequenceId": 8,
                "nodePosition": {
                    "x": 2.0,
                    "y": 0.95,
                    "theta": -0.66,
                    "mapId": "map"
                },
                "actions": []
            }
        ],
        "edges": [
            {
                "edgeId": "edge1",
                "released": true,
                "sequenceId": 1,
                "startNodeId": "node1",
                "endNodeId": "node2",
                "actions": []
            },
            {
                "edgeId": "edge2",
                "released": true,
                "sequenceId": 3,
                "startNodeId": "node2",
                "endNodeId": "node3",
                "actions": []
            },
            {
                "edgeId": "edge3",
                "released": true,
                "sequenceId": 5,
                "startNodeId": "node3",
                "endNodeId": "node4",
                "actions": []
            },
            {
                "edgeId": "edge4",
                "released": true,
                "sequenceId": 7,
                "startNodeId": "node4",
                "endNodeId": "node1",
                "actions": []
            }
        ]
    }'
    ```
  </details>

  <details>
  <summary>4 nodes order with actions</summary>

  ```sh
  mosquitto_pub -h mosquitto -p 1883 -t uagv/v1/OSRF/TB3_1/order -m '
  {
      "orderId": "'$(cat /proc/sys/kernel/random/uuid)'",
      "orderUpdateId": 0,
      "version": "2.0.0",
      "manufacturer": "OSRF",
      "serialNumber": "TB3_1",
      "nodes": [
          {
              "nodeId": "node1",
              "released": true,
              "sequenceId": 0,
              "nodePosition": {
                  "x": 2.0,
                  "y": 0.95,
                  "theta": -0.66,
                  "mapId": "map"
              },
              "actions": []
          },
          {
              "nodeId": "node2",
              "released": true,
              "sequenceId": 2,
              "nodePosition": {
                  "x": 1.18,
                  "y": -1.76,
                  "theta": 0.0,
                  "mapId": "map"
              },
              "actions": [
                  {
                      "actionType": "beep",
                      "actionId": "'$(cat /proc/sys/kernel/random/uuid)'",
                      "actionDescription": "Make a beep noise on node",
                      "blockingType": "NONE",
                      "actionParameters": []
                  }
              ]
          },
          {
              "nodeId": "node3",
              "released": true,
              "sequenceId": 4,
              "nodePosition": {
                  "x": -0.38,
                  "y": 1.89,
                  "theta": 0.0,
                  "mapId": "map"
              },
              "actions": [
                  {
                      "actionType": "beep",
                      "actionId": "'$(cat /proc/sys/kernel/random/uuid)'",
                      "actionDescription": "Make a beep noise on node",
                      "blockingType": "NONE",
                      "actionParameters": []
                  }
              ]
          },
          {
              "nodeId": "node4",
              "released": true,
              "sequenceId": 6,
              "nodePosition": {
                  "x": -0.17,
                  "y": 1.74,
                  "theta": -2.6,
                  "mapId": "map"
              },
              "actions": [
                  {
                      "actionType": "beep",
                      "actionId": "'$(cat /proc/sys/kernel/random/uuid)'",
                      "actionDescription": "Make a beep noise on node",
                      "blockingType": "NONE",
                      "actionParameters": []
                  }
              ]
          },
          {
              "nodeId": "node1",
              "released": true,
              "sequenceId": 8,
              "nodePosition": {
                  "x": 2.0,
                  "y": 0.95,
                  "theta": -0.66,
                  "mapId": "map"
              },
              "actions": [
                  {
                      "actionType": "beep",
                      "actionId": "'$(cat /proc/sys/kernel/random/uuid)'",
                      "actionDescription": "Make a beep noise on node",
                      "blockingType": "NONE",
                      "actionParameters": []
                  }
              ]
          }
      ],
      "edges": [
          {
              "edgeId": "edge1",
              "released": true,
              "sequenceId": 1,
              "startNodeId": "node1",
              "endNodeId": "node2",
              "actions": []
          },
          {
              "edgeId": "edge2",
              "released": true,
              "sequenceId": 3,
              "startNodeId": "node2",
              "endNodeId": "node3",
              "actions": []
          },
          {
              "edgeId": "edge3",
              "released": true,
              "sequenceId": 5,
              "startNodeId": "node3",
              "endNodeId": "node4",
              "actions": []
          },
          {
              "edgeId": "edge4",
              "released": true,
              "sequenceId": 7,
              "startNodeId": "node4",
              "endNodeId": "node1",
              "actions": []
          }
      ]
  }'
  ```
  </details>

  <details>
  <summary>Init position instant action</summary>

  ```sh
  mosquitto_pub -h mosquitto -p 1883 -t uagv/v1/OSRF/TB3_1/instantActions -m '
  {
      "version": "2.0.0",
      "manufacturer": "OSRF",
      "serialNumber": "TB3_1",
      "actions": [
          {
              "actionType": "initPosition",
              "actionId": "'$(cat /proc/sys/kernel/random/uuid)'",
              "blockingType": "NONE",
              "actionParameters": [
                  {
                      "key": "x",
                      "value": "2.0"
                  },
                  {
                      "key": "y",
                      "value": "-0.5"
                  },
                  {
                      "key": "theta",
                      "value": "0.0"
                  },
                  {
                      "key": "mapId",
                      "value": "map"
                  }
              ]
          }
      ]
  }'
  ```
  </details>
