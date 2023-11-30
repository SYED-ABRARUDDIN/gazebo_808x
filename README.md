# Week 9 Gazebo controller

## Abrarudddin Syed (120109997)
 ROS 2 Programming Assignment 'Working with Gazebo'


### Install Dependencies

To, install these dependencies, run the following commands below.
```sh
sudo apt -y install ros-humble-gazebo-ros-pkgs
sudo apt -y install ros-humble-turtlebot3*
sudo apt -y install ros-humble-turtlebot4-desktop
```

### Building and Running
To build and run the simulation, run the following commands.
Firstly, navigate to the source code folder in your workspace ([ros2_ws]/src) and clone the repository.
```sh
# Navigate to the src folder of the workspace:
  cd [ros2_ws]/src
# Clone the repository
  git clone https://github.com/SYED-ABRARUDDIN/gazebo_808x.git
```
# Go back to the ros2 workspace:
  cd ../..
# Make sure there are no dangling dependencies using rosdep
  rosdep install -i --from-path src --rosdistro humble -y
# Build the package:
  colcon build --packages-select gazebo_turtlebot_controller
# Install the package:
  source install/setup.bash

  
```
To run the launch file that executes the simulation, run the following command.
```sh
# Run the Simulation
  ros2 launch gazebo_controller gazebo_turtlebot.launch.py record_bag:=True
```


To replay the bag run the following commands from where you have launch your file.
```sh
# Get more information about the bag file
  ros2 bag info <bag_name>
# Play the bag file
  ros2 bag play <bag_name>
```
An example of a bag file that may be obtained is shown in `results/`.

### Check style guidelines
```bash
#In the package directory
cd ~/ros_ws/src/gazebo_controller

# Cppcheck
$ cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^./build/" ) --check-config > results/cppcheck.txt

# cpplint
$ cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp >  results/cpplint.txt
```

### Buidling Doxygen Documentation
```bash
$ cd ~/ros_ws
#Run the colcon build on the doxygen docs cmake target
$ colcon build --packages-select gazebo_controller --cmake-target docs
