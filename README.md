# WILN - Weather-Invariant Lidar-based Navigation
WILN is a teach-and-repeat framework relying on lidar-based navigation to perform autonomous route repeating. This is the framework that was used in the ["Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned"](https://norlab.ulaval.ca/publications/field-report-ltr/) field report. An overview of the system demonstration is presented in the following video : 

[![SNOW WILN deployment](https://img.youtube.com/vi/W8TdAoeNv4U/0.jpg)](https://www.youtube.com/watch?v=W8TdAoeNv4U)


The WILN system interfaces two libraries/packages: [`libpointmatcher`](https://github.com/norlab-ulaval/libpointmatcher) with [`norlab_icp_mapper`](https://github.com/norlab-ulaval/norlab_icp_mapper) for simultaneous localization and mapping and [`GeRoNa`](https://github.com/cogsys-tuebingen/gerona) for path-following control.

Installation
------------

### Setting up libpointmatcher and the mapper

First, in your local repositories directory, clone and build `norlab_icp_mapper`:

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
    

Then, in your ROS catkin workspace, add the ROS packages to perform SLAM

    cd && cd catkin_ws/src
    git clone git@github.com:norlab-ulaval/norlab_icp_mapper_ros.git
    git clone git@github.com:norlab-ulaval/libpointmatcher_ros.git
    cd .. && catkin_make

### Setting up GeRoNa

GeRoNa is a collection of ROS packages, which can be installed along with their dependencies this way:

First, install the dependencies. For example, from your workspace root directory:

    cd  && cd catkin_ws/src
    git clone https://github.com/cogsys-tuebingen/cslibs_path_planning
    git clone https://github.com/cogsys-tuebingen/cslibs_navigation_utilities
    cd ..

    rosdep install --from-paths -i -r -y src
    catkin_make

Quick start
------------

### Creating a params launch file

Two distinct configuration files are required. The first ones are intended to define the path following parameters, multiple examples can be found in the [`params/controller`](https://github.com/norlab-ulaval/WILN/tree/master/params/controller) directory. For an extensive definition of parameters, please refer to the [GeRoNa wiki](https://github.com/cogsys-tuebingen/gerona/wiki/path-follower-parameters). 

The second ones define the registration parameters and filters for the SLAM algorithm. A working example can be found in the [`params/icp`]() directory. For more details on mapping parameters, please refer to the [norlab_icp_mapper_ros](https://github.com/norlab-ulaval/norlab_icp_mapper_ros) repository.

### Creating a general launch file

General launch files will launch the various nodes required for navigation. They will also import the aforementioned parameters. A [launch file example](https://github.com/norlab-ulaval/WILN/blob/master/launch/warthog.launch) is given.


General operation and services
------------


