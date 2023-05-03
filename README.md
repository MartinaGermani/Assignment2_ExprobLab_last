# Assignment2 of Experimental Robotics Laboratory course
### 1. Brief introduction to the project
In this repository, you can find a robotic simulation for surveillance purposes.

The idea of the software architecture is that at the beginning the robot builds the "semantic" map of the environment by detecting, without moving the base of the robot, all seven markers that are present around it. Once built the map, the goal of the robot is to perform a patrolling of the environment; in particular, it has to move between different locations in order to visit them and stay there for some times, scanning around it by rotating the camera. Moreover, when its battery is low, it has to move in the recharging station in order to recharge.

## 2. Software Components
The software is composed of 6 nodes, each one available in the `scripts/` folder. Moreover there is an `architecture_name_mapper` interface which containes all the names of the parameters, topics and services used in the architecture. 
