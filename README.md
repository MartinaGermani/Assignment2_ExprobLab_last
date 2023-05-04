# Assignment2 of Experimental Robotics Laboratory course
### 1. Brief introduction to the project
In this repository, you can find a robotic simulation for surveillance purposes.

The idea of the software architecture is that at the beginning the robot builds the "semantic" map of the environment by detecting, without moving the base of the robot, all seven markers that are present around it. Once built the map, the goal of the robot is to perform a patrolling of the environment; in particular, it has to move between different locations in order to visit them and stay there for some times, scanning around it by rotating the camera. Moreover, when its battery is low, it has to move in the recharging station in order to recharge.

The ROS-based software architecture is based on the OWL-DL approach to create an ontology of the environment and it uses `SMACH` to implement a Finite State Machine to control the robot behaviour. In particular, the ontology is visible with `Protégé` and the architecture behaviour is based on `ARMOR`.

## 2. Software Components
The ROS-based software architecture is composed of 6 nodes, each one available in the `scripts/` folder. Moreover there is an `architecture_name_mapper` interface which containes all the names of the parameters, topics and services used in the architecture, and also information about the environment. So, with respect the first assignment, I slightly modified the architecture by adding an initial state which handles the marker detection and a reasoning state which decides what to do on the basis of the information which it receives from the other states. 

### The `finite_state_machine` node ###
This node implements the `Finite State Machine` of the architecture which manages the behaviour of the robot. It communicates with the other nodes using ros-services and ros-actions. 
The Finite State Machine is composed by a top level finite state machinem which is composed by an initial state and a resoning one, and by two sub-state machines which manage the robot's motion and its battery. 
The sub-state machine which manages the motion of the robotm includes a state, called Pointing, in which the path is computed and another state, called Controlling, which controls the motion of the robot through the via_points of the path. 
The second sub-state machine, instead, handles the recharging of the robot, in fact, if the robot's battery is low and it is not in the DOC-station, it has to move in it in order to start the recharge, and for doing that it's necessary to plan and navigate the robot there.  

### The `initial_state` node ###
This node implements the `Initial State` which is called by the finite-state machine node. The caller happens through /start service request. 
Initially it loads an empty ontology, whereas then all the informations about the environment are retrived from aruco markers detection by the robot which moves its arm thanks to the MoveIt control node. In this way the ontology is loaded with the corresponding room and doors.

In particular, the `my_moveit node` in src folder allows to implements a predefined motions of the arm. In particular, I set 5 different positions that may be requested:
1. move left the arm
2. move to home position the arm
3. move right the arm
4. move back the arm
5. stop to move

By moving the arm, the robot detects the aruco markers, in particular it retrieves only codes which will be translated in informations about the environment thanks to the `marker_server node`, always in src folder, through /room_info service.

### The `reasoner` node ###
This node implements the `reasoner` state. It is called by the Finite State Machine node through /reason service request. This node reasons about the next-location to be visited and it returns the target location in the /reason service response.

### The `battery` node ###
This node implements the `battery` state. It is called by the Finite State Machine node through /recharge service request. When it is called, it starts a loading bar animation in order to simulate the recharging procedure.

### The `planner` node ###
This node implements the `planner` state, which is called by the Moving sub-state machine through a /control action-client request. It takes the current location and the target one and generate a set of via points to connect them.

### The `controller` node ###
This node implements the `controller` state, which is called through a /plan action-server request. It takes the via points generated and move the robot in the environment.

### 3. Robot and Environment
For the patrolling, I used a `turtlebot3 robot` equipped with a laser, which is essential for the navigation in order to avoid obstacles. Moreover the robot has an arm equipped with a camera mounted on it, which is used for the markers detection. 

The environment, instead, is the same of the first assignment, so it is composed by:
- 7 `doors` (D1, D2, D3, D4, D5, D6);
- 5 `rooms` (E, R1, R2, R3, R4);
- 2 `corridors` (C1, C2)
With respect to the simple implementation of the first assignment, here the environment is 3D and it presents also structural obstacles that the robot has to avoid. Moreover there is an additional small room connected to the room E, in which the robot is spawn initially and in which it has to detect the markers in order to load the semantic map before to start the patrolling. 

### 4. Software Behaviour
After the initial loading of the map of the environment, the robot starts to move between the locations so as to monitor the environment.
In particular, the motion of the robot follows two different protocols, one for the mapping of the unknown environment and one for the navigating from the current location to the desired one. 
For the mapping I used `Gmapping (FastSLAM)`, which is a `Filtering-Based approach` which uses a particle filter in which each particle carries an individual map of the environment. It need the odometry data and a lasers, in fact it subscribes to the topic `/scan` on which the robot publishes the laser and to the topic `/tf`. 
Instead, for the navigation, I used the `MoveBase package` of the ROS Navigation stack, which allows to select a local and a global planner. As global path planning I chose the `navfn`, which uses `Dijkstra’s algorithm` to find a global path with minimum cost between start point and end point. As local path planning, instead, I chose the `dwa`, which depends on the local costmap which provides obstacle information. 

### 5. Commented running

### 6. Installation and running procedure
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


### 7. Limitation and possible improvement
The main limitation of this project is surely the fact that the recharging of the battery is simply simulated by moving the robot where the recharge is supposed to be and by waiting some time; so, a possible improvement consists of making this part more realistic.

### 8. Author and Contact
*Author*: **Germani Martina**

*Contact*: **martina.germani99@gmail.com**
