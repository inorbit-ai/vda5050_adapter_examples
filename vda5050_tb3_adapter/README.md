# VDA5050 TB3 Adapter

ROS2 package with a Python implementation of the adapter for a [Turtlebot 3](https://github.com/ROBOTIS-GIT/turtlebot3/tree/galactic-devel). It contains the following directories:

- **config**: provides a _connector_tb3.yaml_ file to configure the MQTT bridge, controller and adapter parameters.
- **launch**: provides two `.launch.py` files, one for running the `tb3_adapter` node alone and the other to run the whole connector (`mqtt_bridge` and `controller` nodes from the `vda5050_connector` package, as well as the `tb3_adapter` node).

## Running the TB3 adapter

To test the TB3 adapter sample, follow these steps:

- Execute the _simulation_ environment (see [docker environment](../docker/README.md)).

- Inside the _development_ container:

  ```sh
  colcon build && source install/setup.bash
  ```

  ```sh
  ros2 launch vda5050_tb3_adapter connector_tb3.launch.py
  ```

- Attach a new terminal to the _development_ container (see [docker environment](../docker/README.md)).

- Within the container, test the already created order / actions under the _test/integration_ folder of the TB3 package:

  ```sh
  cd src/vda5050_tb3_adapter/test/integration
  ```

  ```sh
  python3 order1.py # Send robot to navigate
  ```