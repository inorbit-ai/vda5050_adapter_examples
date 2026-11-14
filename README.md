# VDA5050 Adapter Examples

## Description

This repository contains useful packages and tools to work with the [vda5050_connector](https://github.com/inorbit-ai/ros_amr_interop/tree/galactic-devel/vda5050_connector)
package. These resources provide a starting point to implement your own VDA5050 Adapter.

## Examples

### Docker

The **docker** directory contains a build / deployment configuration to easily work and test the
`vda5050_connector` package, your custom adapter and their dependencies. Check its [README](/docker/README.md)
for more details.

### VDA5050 TB3 Adapter

The **vda5050_tb3_adapter** is a ROS2 package with a python implementation of the adapter. Check its
[README](/vda5050_tb3_adapter/README.md) for more details.

### VDA5050 Template package (C++)

The **vda5050_template_package** is a ROS2 package with a C++ template to implement a custom VDA5050
adapter. Check its [README](/vda5050_template_package/README.md) for more details.

## Contributing

Please see the [CONTRIBUTING](CONTRIBUTING.md) document.

## License

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](LICENSE)
