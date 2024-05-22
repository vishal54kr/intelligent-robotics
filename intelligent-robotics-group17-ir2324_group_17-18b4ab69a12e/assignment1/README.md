## Group 17:
- *Marco Calì*, marco.cali.2@studenti.unipd.it
- *Anne Linda Antony Sahayam*, annelinda.antonysahayam@studenti.unipd.it
- *Vishal Kumar*, vishal.kumar@studenti.unipd.it

## Problem Statement

The task is to implement a routine that enables the Tiago robot, to navigate within a given environment. The environment consists of two rooms with movable obstacles and a narrow space. Tiago needs to navigate from the starting pose to a user-defined pose. Once at the desired pose, the robot should detect movable obstacles, such as cylindrical tables, and print their positions on the screen. Static obstacles that are part of the map, such as walls and shelves, must not be detected as obstacles.

## Implementation Structure

### 1. User Input

The user provides the desired pose via the command line when running the client.

### 2. Action Client/Server Structure

- **Action Client:**
  - Receives input from the user.
  - Calls the action server to execute tasks.
  - Implements callbacks to receive feedback from the action server.
  - Prints the current status of the task in the terminal.

- **Action Server:**
  - Receives input from the action client.
  - Executes tasks, including navigation and obstacle detection.
  - Sends the final list of obstacle positions as a result to the action client.
  - Provides feedback to the action client, reflecting the current status of the robot (e.g., stopped, moving, obstacle detection in final pose, detection finished, etc)

### 3. Extra Points
Implement a control law to navigate through the narrow corridor using the laser scan data.

## How to run
Ensure to have installed ROS Noetic and tiago_public-noetic in your machine (this is not necessary if running on VLAB).
Create a new workspace and clone the project

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://MarcoCali0@bitbucket.org/intelligent-robotics-group17/ir2324_group_17.git
git clone https://github.com/PieroSimonet/tiago_iaslab_simulation.git
cd ..
catkin build
source ~/catkin_ws/devel/setup.bash 
```

Launch the Tiago environment and simulation using the following commands:

```bash 
roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=robotics_library
roslaunch tiago_iaslab_simulation navigation.launch
```

Now, in one terminal, run:

```bash
rosrun assignment1 navigation_server
```

Open another terminal, and run the client giving as input the desired Cartesian coordinates and the orientation in degrees: 

```bash 
rosrun assignment1 navigation_client x y theta
```

For example:

```bash
rosrun assignment1 navigation_client 12.0 -2.0 0.0
```

To run the custom corridor navigation node:
```bash
rosrun assignment1 navigation_corridor
```