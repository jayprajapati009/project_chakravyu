
# Project Chakravyu

[![codecov](https://codecov.io/gh/jayprajapati009/project_chakravyu/branch/main/graph/badge.svg?token=0C30FZ9SC6)](https://codecov.io/gh/jayprajapati009/project_chakravyu)
[![Build Status](https://github.com/jayprajapati009/project_chakravyu/actions/workflows/build_and_coveralls.yml/badge.svg)](https://github.com/ayprajapati009/project_chakravyu/actions/workflows/build_and_coveralls.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## Authors

|Name|ID|Email|
|:---:|:---:|:---:|
|Jay Prajapati|119208625|jayp@umd.edu|
|Shail Shah|119340547|sshah115@umd.edu|
|Shantanu Parab|119347539|sparab@umd.edu|

***Team Id: Group - 12***

## Introduction

Simple robots working together in a multi-robot system to accomplish predetermined objectives. Robots in these systems are much less powerful when acting alone, but the actual power comes from their ability to work together. Industrial robots with several uses can automate a variety of manufacturing processes. A single industrial robot frequently has the capacity to carry out several different kinds of tasks. The scope of this study encompasses all of robotics' related subfields, including robotic control systems, planning, and perception. These systems are widely used in warehouse robotics, cooperative robotics, swarm robotics, unmanned aerial vehicles, etc. In this project, we'll create a software module that uses C++ and ROS to control more than twenty ROS TurtleBots at once to create geometric patterns.

## Dependencies

- OS: Ubuntu Linux 22.04
- ROS Version: ROS2 Humble Hawksbill
- Python: Python 3

## System Architecture

The fleet of robots while performing the prescribed task of aligning to form a geometric pattern might collide during movement. Communication error due to frequency mismatch between nodes might lead to robots going haywire and can disrupt the process. Disconnection of master node or slave robots from the network might lead to inconsistent geometric patterns being formed or a complete standstill of the entire process. The architecture of our system will be as follows,

![System Architecture](https://github.com/jayprajapati009/project_chakravyu/blob/iteration_2/documents/updated_system_architecture.png)

## Deliverables

|Deliverable|Link|
|:---|:---:|
|Demo Video|[Link](https://mail.google.com/mail/u/2/#inbox/FMfcgzGrbbvPPhzvpBTXZQNtKCxXJLjw?projector=1)|
|Presentation|[Link](https://drive.google.com/drive/folders/1N3aHFS-1haXkBilldU6IvjWTysK8GAJ8?usp=share_link)|
|UML Class Diagram|[Link](https://github.com/jayprajapati009/project_chakravyu/blob/iteration_1/UML/initial/class_diagram.png)|
|AIP Backlog and Worklog Sheet|[Link](https://docs.google.com/spreadsheets/d/1fCrZ5zCcu7wbSNEzoXJNjJSVYKHJ8yOg8b6y1aFy0Is/edit?usp=sharing)|
|Sprint and Review Meeting Notes|[Link](https://docs.google.com/document/d/1zADA51S8-DCuGPjZB7dvrBzD6DiS--uvvF-nh4I-Mvw/edit?usp=sharing)|
|Project Proposal Document|[link](https://github.com/jayprajapati009/project_chakravyu/blob/iteration_1/documents/Project_Chakravyu_Proposal.pdf)|

## SystemSetup

In ubuntu linux 22.04 LTS version Install ```ROS2 Humble```. The Installation documentation can be found [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

After successfully setting up the ROS2 Humble, create a workspace.
Source the Ros2,

```sh
source /opt/ros/humble/setup.bash
```

Create the directory

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone the repository,

```sh
git clone https://github.com/jayprajapati009/project_chakravyu.git
```

Now, check the dependencies,

```sh
# cd if you're still in the ``src`` directory with the ``project_chakravyu`` clone
cd ..
rosdep install -i --from-path src --rosdistro humble -y
```

and build the package,

```sh
colcon build --packages-select project_chakravyu
```

## Run the module

Once the package is build successfully, to run the module.

```sh
ros2 launch project_chakravyu gazebo_launch.py node_count:=20
```

The ```node_count``` parameter is number of robots user wants to launch. Default value is 10.
In another terminal run after all robots are spawned, to align the robots in a circle

```sh
ros2 run project_chakravyu master 20
```

here again, ```20``` is the number of robots the user wants to spawn.

## Cpplint and Cppcheck

To check the Cpplint and Cppcheck results, check ```results/``` directory. To run the check,

```sh
sh cpplint.sh
sh cppcheck.sh
```

## Known bugs

- While launch 20+ robots at once, ensure that your system has updated GPU drivers and they are proprietary and tested.
- If the Gazebo Simulation fails, restart the launch file. Check whether the previous  ```gzserver``` is not running in background, using ```ps -a``` in terminal.
