# ROS-Dash
Displaying ROS Data in a Web-based dashboard using Python Dash Framework. 

An example is provided that loads custom ECG data from nested CSV files to publish to a topic. The dashboard demonstrates subscribing to the topic and displaying published data using Pandas dataframes. As an additional component, a .png map generated by a turtlebot3 in a gazebo environment running turtlebot3_slam is live updated.

![image](https://github.com/Chttan/ROS-Dash/assets/26287515/63598aa1-9e09-4237-8b0b-d2a1256230ec)

## Setup
Initially developed on a VM running Ubuntu 20.04 with ROS 1 Noetic. Below are the steps used to setup the VM from a fresh install.

```
sudo apt update
sudo apt upgrade
```

### ROS 1 Install
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop-full
```
### Configure .bashrc
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# This solves a problem with a VM not drawing map objects in gazebo
echo "export SVGA_VGPU10=0" >> ~/.bashrc

source ~/.bashrc
```

### Additional Dependencies

```
sudo apt install python3-vcstool
sudo apt install python3-catkin-tools python3-catkin-lint python3-pip

sudo apt install python3-rosdep
sudo rosdep init
rosdep update

python3 -m pip install osrf-pycommon
```

### Catkin Workspace

```
# Feel free to substitute your own path for `ROS-Task`
mkdir ROS-Task/src
cd ROS-Task

# Initilize the catkin workspace
catkin init
catkin config --extend /opt/ros/noetic
catkin config -DCMAKE_BUILD_TYPE=Release

# Additional modification to .bashrc for easy environment setup on new terminal launch
echo "source ~/ROS-Task/devel/setup.bash" >> ~/.bashrc
```

### Gazebo Install

https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install

```
curl -sSL http://get.gazebosim.org | sh

# Test the install
gazebo
```

### Turtlebot Simulation Setup

```
cd ROS-Task/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
cd ..

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd turtlebot3
git checkout noetic-devel
cd ../..

rosdep install -q -y -r --from-paths src --ignore-src
catkin_make install

# Additional modification to .bashrc for easy environment setup on new terminal launch
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc

sudo apt install ros-noetic-dynamixel-sdk ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3
```

### Dash+Additional Python Dependency Setup

```
pip install dash
pip install pandas
```

## Running

Example to see everything in action. In 5 separate terminals, run:

1. Simulation Environment
   
`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

2. Bio Data Publisher
   
`python3 BioPub.py`

3. Turtlebot SLAM Capabilities
   
`roslaunch turtlebot3_slam turtlebot3_slam.launch`

4. Web Dashboard
   
`python3 web-dash.py`

5. Manual keyboard control of Turtlebot
    
`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`

Navigate to [127.0.0.1:8050](127.0.0.1:8050). Steer the turtlebot using teleop to map out the world. Observe the map updating in ROS-Dash compared to RViz. Press the button in the web dashboard to subscribe to the Bio Data publisher.
