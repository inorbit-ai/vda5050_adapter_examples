# Docker scripts

This directory contains two Dockerfiles:

- **development**: useful to compile, run and test the connector package with its adapter.

- **simulation**: launches a TB3 Gazebo simulation to test navigation commands and VDA actions
  using the adapter.

These docker images can be built and run using the **docker-compose** file. The compose allows:

- Manipulating the execution of three containers: _development_, _simulation_ and _mosquitto_ (MQTT broker).
- `DISPLAY` capabilities.
- Creating a shared network between the containers.
- Mounting the example adapter packages.

To work with it, there are two bash scripts that launch, attach and stop the containers:

- Run _development_ environment:

  ```sh
  ./setup.sh # Only development and mosquitto containers.
  ```

- Run _simulation_ environment:

  ```sh
  ./setup-sim.sh # Run the three containers.
  ```

  **Note**: the simulation logs will be flooded with `discarding message because the queue is full`
  messages. This is expected, and they will go away after setting simulated robot initial pose.

These scripts automatically attach to the _development_ container. To attach another terminal to a
given container (`development`, `simulation` or `mosquitto`) use:

```sh
cd deployment && docker compose exec <container> bash
```

Inside the _development_ container, you can directly compile the ROS2 code (`colcon build`) under
the `dev_ws` folder, open tools like `rqt` or even send a message via MQTT.
