# Assignment2 of Experimental Robotics Laboratory course
### 1. Brief introduction to the project
In this repository, you can find a robotic simulation for surveillance purposes.

The idea of the software architecture is that at the beginning the robot builds the "semantic" map of the environment by detecting, without moving the base of the robot, all seven markers that are present around it. Once built the map, the goal of the robot is to perform a patrolling of the environment; in particular, it has to move between different locations in order to visit them and stay there for some times, scanning around it by rotating the camera. Moreover, when its battery is low, it has to move in the recharging station in order to recharge.

The ROS-based software architecture is based on the OWL-DL approach to create an ontology of the environment and it uses `SMACH` to implement a Finite State Machine to control the robot behaviour. In particular, the ontology is visible with `Protégé` and the architecture behaviour is based on `ARMOR`.

## 2. Software Components
The ROS-based software architecture is composed of 6 nodes, each one available in the `scripts/` folder. Moreover there is an `architecture_name_mapper` interface which containes all the names of the parameters, topics and services used in the architecture, and also information about the environment. 

### The `finite_state_machine` node ###
This node implements the `Finite State Machine` of the architecture which manages the behaviour of the robot. It communicates with the other nodes using ros-services and ros-actions. The Finite State Machine is composed by a top level finite state machine and by two sub-state machines which manage the robot's motion and its battery. 

### The `initial_state` node ###
This node implements the `Initial State` which is called by the finite-state machine node. The caller happens through /start service and request. 
Initially it loads an empty ontology, whereas then all the informations about the environment are retrived from aruco markers detection by the robot which moves its arm thanks to the MoveIt control node. In this way the ontology is loaded with the corresponding room and doors.
### The `reasoner` node ###

### The `battery` node ###

### The `planner` node ###

### The `controller` node ###

### 3. Software Behaviour

### 4. Commented running

### 5. Installation and running procedure
This architecture runs on ROS noetic and it has been developed with the Docker image, that you can find [here](https://hub.docker.com/repository/docker/carms84/exproblab). 

For the implementation to work, it's necessary to:
- install [aRMOR](https://github.com/EmaroLab/armor) and the [ArmorPy API](https://github.com/EmaroLab/armor_py_api);
- install `smach`: `sudo apt-get install ros-noetic-smach-ros`;
- install `MoveIt`: `sudo apt-get install ros-noetic-moveit`. It is an open-source robotic manipulation platform which allows to develop complex application, to plan trajectories that the robot's joints should follow and to execute these trajectories;
- install `xterm`: `sudo apt-get install -y xterm`;
- clone [aruco](https://github.com/pal-robotics/aruco_ros) package, which provides real-time marker based 3D pose estimation using AR markers
- clone [MoveBase Navigation](https://github.com/ros-planning/navigation) package, which is part of the ROS Navigation Stack, and it allows to plan, execute and control robot's trajectories;
- clone [SLAM gmapping](https://github.com/ros-perception/slam_gmapping) package, which stands for Simultaneous Localization and Mapping and where gmapping is a specific SLAM algorithm which uses laser range-finder data to build the map.


Once you have all the installation required, you have to do the following steps:
- ```git clone https://github.com/MartinaGermani/Assignment2_ExprobLab_last.git ```
- go in the `scripts/` folder and run `chmod +x *`
- run `catkin build` in the workspace
- once the workspace is built, open two terminals and execute the following commands:

```roslaunch patrol_robot demo_assignment.launch```

```roslaunch patrol_robot run.launch```

### 6. Working hypothesis and environment

### 7. Limitations


### 8. Author and Contact
*Author*: **Germani Martina**

*Contact*: **martina.germani99@gmail.com**
