# WILN - Weather-Invariant Lidar-based Navigation

[![DOI](https://zenodo.org/badge/DOI/10.55417/fr.2022050.svg)](https://doi.org/10.55417/fr.2022050)
[![ROS1](https://img.shields.io/badge/ROS1-melodic-blue?labelColor=blue&logo=ROS)](http://wiki.ros.org/melodic)
[![ROS2](https://img.shields.io/badge/ROS2-humble-blue?labelColor=blue&logo=ROS)](https://docs.ros.org/en/humble)

WILN is a teach-and-repeat framework relying on lidar-based navigation to perform autonomous route repeating. This is the framework that was used in the ["Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned"](https://norlab.ulaval.ca/publications/field-report-ltr/) field report. An overview of the system demonstration is presented in the following video :

[![SNOW WILN deployment](https://img.youtube.com/vi/W8TdAoeNv4U/0.jpg)](https://www.youtube.com/watch?v=W8TdAoeNv4U)

The WILN system interfaces two libraries/packages: [`libpointmatcher`](https://github.com/norlab-ulaval/libpointmatcher) with [`norlab_icp_mapper`](https://github.com/norlab-ulaval/norlab_icp_mapper) for simultaneous localization and mapping and [`GeRoNa`](https://github.com/cogsys-tuebingen/gerona) for path-following control.

## Installation

### Setting up libpointmatcher and the mapper

First, in your local repositories directory, clone and build `norlab_icp_mapper`:

```sh
cd && cd repos/
git clone git@github.com:ethz-asl/libpointmatcher.git  && cd libpointmatcher/
mkdir build/ && cd build/
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 6
sudo make install
cd && cd repos
git clone git@github.com:norlab-ulaval/norlab_icp_mapper.git && cd norlab_icp_mapper/
mkdir build/ && cd build/
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j 6
sudo make install
```

Then, in your ROS catkin workspace, add the ROS packages to perform SLAM

```sh
cd && cd catkin_ws/src
git clone git@github.com:norlab-ulaval/norlab_icp_mapper_ros.git
git clone git@github.com:norlab-ulaval/libpointmatcher_ros.git
git clone git@github.com:norlab-ulaval/pointcloud_motion_deskew.git
git clone git@github.com:norlab-ulaval/odom_to_pose_converter.git
cd .. && catkin_make
```

### Setting up GeRoNa

GeRoNa is a collection of ROS packages, which can be installed along with their dependencies this way:

First, install the dependencies. For example, from your workspace root directory:

```sh
cd  && cd catkin_ws/src
git clone https://github.com/cogsys-tuebingen/cslibs_path_planning
git clone https://github.com/cogsys-tuebingen/cslibs_navigation_utilities
sudo apt install libalglib-dev
git clone https://github.com/cogsys-tuebingen/gerona.git
cd ..

rosdep install --from-paths -i -r -y src
catkin_make
```

## Quick start

General note : This assumes that you already have standard [timestamped lidar scan](https://github.com/norlab-ulaval/ros_rslidar), [imu](https://github.com/ethz-asl/ethzasl_xsens_driver) and general odometry nodes running.

### Creating a params launch file

Two distinct configuration files are required. The first ones are intended to define the path following parameters, multiple examples can be found in the [`params/controller`](https://github.com/norlab-ulaval/WILN/tree/master/params/controller) directory. For an extensive definition of parameters, please refer to the [GeRoNa wiki](https://github.com/cogsys-tuebingen/gerona/wiki/path-follower-parameters).

The second ones define the registration parameters and filters for the SLAM algorithm. A working example can be found in the [`params/icp`](params/) directory. For more details on mapping parameters, please refer to the [norlab_icp_mapper_ros](https://github.com/norlab-ulaval/norlab_icp_mapper_ros) repository.

### Creating a general launch file

General launch files will launch the various nodes required for navigation. They will also import the aforementioned parameters. A [launch file example](https://github.com/norlab-ulaval/WILN/blob/master/launch/warthog.launch) is given.

## General operation and services

The framework is divided into two phases: teach and repeat. During the teach phase, an operator drives the robot along a desired route. The robot simultaneously localizes and builds a map of the environment. All robot poses are logged and represent the reference trajectory.

During the repeat phase, the robot repeats a given route. This route can either be a map that has just been recorder or a loaded map that was saved on disk. During this phase, the system registers point clouds to the map to localize but does not update the map.

The following table lists the various ROS services that enable the teach-and-repeat framework:

| Service name | Description | Parameters |
| :----------- | :---------- | :--------- |
| /start_recording | Starts recording poses to build the reference map (cannot be called if another trajectory is already loaded). | None |
| /stop_recording | Stops the trajectory recording (cannot be called is the recording was not started). | None                 |
| /clear_trajectory | Clears the current trajectory from active memory. | None                 |
| /play_trajectory | Starts the repeat phase. The robot will repeat the trajectory backwards if it is located at it's end and the system supports both forwards or reverse motion. | None |
| /play_loop_trajectory | Starts the repeat phase for a loop trajectory. | nbLoops (uint32)     |
| /cancel_trajectory | Cancels the current repeat phase in the event of system failure. | None  |
| /save_ltr | Saves the current map and trajectory in a `.ltr` file. If no directory is specified, the file will be saved in the `home/<user>/.ros` directory. | `file_name` (string) |
| /load_ltr | loads a specicied `.ltr` file (cannot be executed if a map/trajectory is already loaded). If not directory is specified, will load from the `home/<user>/.ros` directory. | `file_name` (string) |

## Citing

If you use wiln in an academic context, please cite [our article](https://doi.org/10.55417/fr.2022050):

```bibtex
@article{Baril2022,
  doi = {10.55417/fr.2022050},
  url = {https://doi.org/10.55417/fr.2022050},
  year = {2022},
  month = mar,
  publisher = {Field Robotics Publication Society},
  volume = {2},
  number = {1},
  pages = {1628--1660},
  author = {Dominic Baril and Simon-Pierre Desch{\^{e}}nes and Olivier Gamache and Maxime Vaidis and Damien LaRocque and Johann Laconte and Vladim{\'{\i}}r Kubelka and Philippe Gigu{\`{e}}re and Fran{\c{c}}ois Pomerleau},
  title = {Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned},
  journal = {Field Robotics}
}
```
