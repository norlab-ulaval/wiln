# WILN - Weather-Invariant Lidar-based Navigation
WILN is a teach-and-repeat framework relying on lidar-based navigation to perform autonomous route repeating. This is the framework that was used in the ["Kilometer-scale autonomous navigation in subarctic forests: challenges and lessons learned"](https://norlab.ulaval.ca/publications/field-report-ltr/) field report. An overview of the system demonstration is presented in the following video : 

[![SNOW WILN deployment](https://img.youtube.com/vi/W8TdAoeNv4U/0.jpg)](https://www.youtube.com/watch?v=W8TdAoeNv4U)


The WILN system interfaces two libraries/packages: [`libpointmatcher`](https://github.com/norlab-ulaval/libpointmatcher) with [`norlab_icp_mapper`](https://github.com/norlab-ulaval/norlab_icp_mapper) for simultaneous localization and mapping and [`GeRoNa`](https://github.com/cogsys-tuebingen/gerona) for path-following control.

Setting up libpointmatcher norlab_icp mapper
------------

First, in your local repositories directory, clone and build `norlab_icp_mapper`:

    cd && cd repos/
    git clone git@github.com:norlab-ulaval/libpointmatcher.git
    mkdir build/ && cd build/
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j 6
    sudo make install
    cd && cd repos
    git clone https://github.com/norlab-ulaval/norlab_icp_mapper && cd norlab_icp_mapper/
    mkdir build/ && cd build/
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j 6
    sudo make install
    

Then, in your ROS catkin workspace, add the ROS packages to perform SLAM

    cd && cd catkin_ws/src
    git clone git@github.com:norlab-ulaval/norlab_icp_mapper_ros.git
    git clone git@github.com:norlab-ulaval/libpointmatcher_ros.git
    cd .. && catkin_make

Setting up GeRoNa
------------

GeRoNa is a collection of ROS packages, which can be installed along with their dependencies this way:

First, install the dependencies. For example, from your workspace root directory:

    cd  && cd catkin_ws/src
    git clone https://github.com/cogsys-tuebingen/cslibs_path_planning
    git clone https://github.com/cogsys-tuebingen/cslibs_navigation_utilities
    cd ..

    rosdep install --from-paths -i -r -y src
    catkin_make

