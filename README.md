# loam_velodyne

# 1. Installation ROS indigo + Ubuntu 14.04
If you don't have a rosbuild workspace, set up one with:

    sudo apt-get install python-rosinstall
    mkdir ~/rosbuild_ws
    cd ~/rosbuild_ws
    rosws init . /opt/ros/indigo
    mkdir package_dir
    rosws set ~/rosbuild_ws/package_dir -t .
    echo "source ~/rosbuild_ws/setup.bash" >> ~/.bashrc
    bash
    cd package_dir

In your ROS package path, clone the repository:

    git clone https://github.com/sebdi/loam_velodyne.git



# 2. How to see robot city results from Ji?
Well, you have to get his rosbag file at:

    https://www.dropbox.com/s/ywne8e0rih4ikuz/robot_city_bridge.bag?dl=0

Then, add the filepath in main_robot_city.cpp. You can see the visualization with

    rviz -d loam_continuous.rviz

