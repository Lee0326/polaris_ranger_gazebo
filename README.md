# polaris_ranger_gazebo
## Install dependencies:

1. Add ROS to sources.list

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

2. Install gazebo with ROS

```
sudo apt-get install ros-melodic-desktop

# Source ROS
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# Install Gzebo9
sudo apt install ros-melodic-gazebo9*
```

3. Initialize rosdep

```
rosdep init
rosdep update
```

4. Install catkin

```
sudo apt-get install ros-melodic-catkin python-catkin-tools
```

## Run the simulation

A car simulation package in gazebo. Run the following command to build it.

```
git clone https://github.com/Lee0326/polaris_ranger_gazebo.git
cd polaris_ranger_gazebo
catkin_make
source devel/setup.bash
```

Run the following command to launch the simulation world with three vehicles:

```
roslaunch platoon_simulation patoon_control.launch
```

You will see the three vehicles in a platoon:

![Yo9YuT.jpg](https://s1.ax1x.com/2020/05/20/Yo9YuT.jpg)

Open another terminal and run the platoon control node

```
cd polaris_ranger_gazebo
source devel/setup.bash
rosrun platoon_control platoon_controller
```

