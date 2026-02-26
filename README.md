# VINS-Multi - Easy ROS2 Setup (With additional configs for custom VOXL2 Starling2 Max)

This fork has modified the VINS-Multi multi-ver branch to be a ros2 only installation with very simple installation steps.

## Installation

1. Install vins_core by running the setup_script.sh, or by performing the steps in the shell script yourself one by one. If you run the installation line by line yourself, make sure to first cd into vins_core.

2. Run a colcon build in the ros2 workspace as usual.

It should be that simple. Note that the original launch file has been modified to launch with the voxl-starling2-max configuration included in the folder config/.

Note also that all naming conventions in the source files and header files have been modified from vins_estimator_ros2 -> vins_multi_ros2. This is a minor adjustment which has no purpose other than maintaining a cleaner namespace throughout this package.

For original README, check [VINS-Multi](https://github.com/HKUST-Aerial-Robotics/VINS-Multi)
