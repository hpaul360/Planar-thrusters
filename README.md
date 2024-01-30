# UAV Planar Thrusters Add-On

ROS1 Package for a UAV with Three Planar Thrusters as Add-On

## Overview

This ROS1 package extends the capabilities of a UAV by adding three planar thrusters. The thrusters are strategically positioned to provide additional control and maneuverability along the horizontal plane. This package integrates with the existing UAV control system to enable planar thrust adjustments.

## Features

- **Planar Thrusters:** Three additional thrusters attached to the UAV, designed for planar control.
- **Enhanced Maneuverability:** The planar thrusters enable enhanced control and maneuverability in the horizontal plane.
- **ROS1 Integration:** Seamless integration with the ROS1 ecosystem for easy configuration and control.

## Prerequisites

- ROS1 (Robot Operating System 1)
- Compatible UAV platform

## Installation

1. Clone this repository into your ROS1 workspace:

    ```bash
    git clone https://github.com/hpaul360/Planar-thrusters.git
    ```

2. Build the package:

    ```bash
    catkin_make
    ```

3. Source the ROS1 setup file:

    ```bash
    source devel/setup.bash
    ```

## Usage

1. Ensure your UAV is equipped with the planar thrusters add-on.
2. Launch the ROS1 node for the planar thrusters:

    ```bash
    roslaunch planar_thrusters control.launch
    ```

3. The UAV is now equipped with enhanced planar control capabilities.

## License

This software is released under the MIT License. See the [LICENSE](LICENSE) file for details.

## Issues and Contributions

Report any issues or contribute to the development of this package on [GitHub](https://github.com/hpaul360/uav_planar_thrusters).

## Author

- [Hannibal Paul](https://github.com/hpaul360)

## Contact

For questions or further assistance, feel free to contact the author [Hannibal Paul](https://hannibalpaul.com/).
