# Project Chakravyu

[![codecov](https://codecov.io/gh/jayprajapati009/my-ros2-project_chakravyu-exp/branch/main/graph/badge.svg?token=KRAHD3BZP7)](https://codecov.io/gh/jayprajapati009/project_chakravyu)
[![Build Status](https://github.com/jayprajapati009/project_chakravyu/actions/workflows/build_and_coveralls.yml/badge.svg)](https://github.com/ayprajapati009/project_chakravyu/actions/workflows/build_and_coveralls.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

# Authors

|Name|ID|Email|
|---|---|---|
|Jay Prajapati|119208625|jayp@umd.edu|
|Shail Shah|119340547|sshah115@umd.edu|
|Shantanu Parab|119347539|sparab@umd.edu|

Team Id: Group - 12

# Introduction

<div style="text-align: justify">
Simple robots working together in a multi-robot system to accomplish predetermined objectives. Robots in these systems are much less powerful when acting alone, but the actual power comes from their ability to work together. Industrial robots with several uses can automate a variety of manufacturing processes. A single industrial robot frequently has the capacity to carry out several different kinds of tasks. The scope of this study encompasses all of robotics' related subfields, including robotic control systems, planning, and perception. These systems are widely used in warehouse robotics, cooperative robotics, swarm robotics, unmanned aerial vehicles, etc. In this project, we'll create a software module that uses C++ and ROS to control more than twenty ROS TurtleBots at once to create geometric patterns.
</div>

# Dependencies

- OS: Ubuntu Linux 22.04
- ROS Version: ROS2 Humble Hawksbill

# System Architecture

The fleet of robots while performing the prescribed task of aligning to form a geometrical pattern might collide during movement. Communication error due to frequency mismatch between nodes might lead to robots going haywire and can disrupt the process. Disconnection of master node or slave robots from the network might lead to inconsistent geometric patterns being formed or a complete standstill of entire process. The architecture of our system will be as follows,


![System Architecture](https://github.com/jayprajapati009/project_chakravyu/blob/iteration_1/documents/proposed_system_architecture.png)

# Deliverables (Phase - 1)

|Deliverable|Link|
|:---|:---:|
|Project Proposal Document|[link](https://github.com/jayprajapati009/project_chakravyu/blob/iteration_1/documents/Project_Chakravyu_Proposal.pdf)|
|UML Class Diagram|[Link](https://github.com/jayprajapati009/project_chakravyu/blob/iteration_1/UML/initial/class_diagram.png)|
|AIP Backlog and Worklog Sheet|[Link](https://docs.google.com/spreadsheets/d/1fCrZ5zCcu7wbSNEzoXJNjJSVYKHJ8yOg8b6y1aFy0Is/edit?usp=sharing)|
|Sprint and Review Meeting Notes|[Link](https://docs.google.com/document/d/1zADA51S8-DCuGPjZB7dvrBzD6DiS--uvvF-nh4I-Mvw/edit?usp=sharing)|




