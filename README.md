# VINS-Multi - Easy ROS2 Setup (With additional configs for custom VOXL2 Starling2 Max)

This fork has modified the VINS-Multi multi-ver branch to be a ros2 only installation with very simple installation steps.

## Installation

0. Prerequisite: Ceres Solver. Run the shell file _install_ceres_solver.sh_ or run the commands line by line yourself. I use the main branch from the ceres solver github as it is stable on Ubuntu 22.04 and above. ceres solver with tags 2.1.0 and 2.2.0 are quite unstable, so do not install those.

1. Install vins\_core by running *setup_script.sh*, or by performing the steps in the shell script yourself. If you run the installation line by line yourself, make sure to first cd into vins_core.

2. On line 8 in the CMakeLists.txt change the path to your vins core install folder: *path/to/vins_multi/vins_core/install/* or wherever you chose to build vins core.

2. Run a colcon build in the ros2 workspace as usual.

It should be that simple. Note that the original launch file has been modified to launch with the voxl-starling2-max configuration included in the folder config/.

Note also that all naming conventions in the source files and header files have been modified from vins_estimator_ros2 -> vins_multi_ros2. This is a minor adjustment which has no purpose other than maintaining a cleaner namespace throughout this package.

For building with a docker environment it is recommended to have vins core in a thirdparty folder that you can build as part of the image. Since vins_core will likely not be modified, this reduces build time and allows faster development in ros2.

For original README, check [VINS-Multi](https://github.com/HKUST-Aerial-Robotics/VINS-Multi)
