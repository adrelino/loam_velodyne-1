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
