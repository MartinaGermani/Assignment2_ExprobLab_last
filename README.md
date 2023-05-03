# Assignment2 of Experimental Robotics Laboratory course
### 1. Brief introduction to the project
In this repository, you can find a robotic simulation for surveillance purposes.

The idea of the software architecture is that at the beginning the robot builds the "semantic" map of the environment by detecting, without moving the base of the robot, all seven markers that are present around it. Once built the map, the goal of the robot is to perform a patrolling of the environment; in particular, it has to move between different locations in order to visit them and stay there for some times, scanning around it by rotating the camera. Moreover, when its battery is low, it has to move in the recharging station in order to recharge.

The ROS-based software architecture is based on the OWL-DL approach to create an ontology of the environment and it uses `SMACH` to implement a Finite State Machine to control the robot behaviour. In particular, the ontology is visible with `Protégé` and the architecture behaviour is based on `ARMOR`.

## 2. Software Components
The ROS-based software architecture is composed of 6 nodes, each one available in the `scripts/` folder. Moreover there is an `architecture_name_mapper` interface which containes all the names of the parameters, topics and services used in the architecture, and also information about the environment. 

### The `finite_state_machine` node ###

### The `initial_state` node ###

### The `reasoner` node ###

### The `battery` node ###

### The `planner` node ###

### The `controller` node ###


### 3. Installation and running procedure
This architecture runs on ROS noetic and it has been developed with the Docker image: you can find [here](https://hub.docker.com/repository/docker/carms84/exproblab) the instruction. 
For the implementation to work, it's necessary to:
- install [aRMOR](https://github.com/EmaroLab/armor) and the [ArmorPy API](https://github.com/EmaroLab/armor_py_api).
- install `smach`: `sudo apt-get install ros-noetic-smach-ros`
Once you have all the installation required, you have to do the following steps:

- ```git clone https://github.com/MartinaGermani/ExprobLab_Assignment1.git ```
- go in the `scripts/` folder and run `chmod +x *`
- run `catkin build` in the workspace
- once the workspace is built, open two terminals and execute the following commands:

```roslaunch patrol_robot demo_assignment.launch```

```roslaunch patrol_robot run.launch```
