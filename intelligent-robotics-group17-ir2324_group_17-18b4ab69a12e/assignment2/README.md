## Group 17:
- *Marco Calì*, marco.cali.2@studenti.unipd.it
- *Anne Linda Antony Sahayam*, annelinda.antonysahayam@studenti.unipd.it
- *Vishal Kumar*, vishal.kumar@studenti.unipd.it

## Problem Statement

The task is to implement a routine that enables the Tiago robot, to navigate within a given environment and perform object detection and manipulation. Tiago needs to navigate from the starting pose to a specific pose to pick an object. Once at the desired pose, the robot should detect obstacles, such as cylindrical tables, and should efficiently perform pick and place routines.


## How to run
Ensure to have installed ROS Noetic and tiago_public-noetic in your machine (this is not necessary if running on VLAB).
Create a new workspace and clone the project

```bash
start_tiago
source /opt/ros/noetic/setup.bash
source /tiago_public_ws/devel/setup.bash
cd ~/catkin_ws/src
source ~/catkin_ws/devel/setup.bash
catkin build
```
To Launch the Tiago environment and simulation use the following command:

```bash 
roslaunch tiago_iaslab_simulation start_simulation.launch world_name:=ias_lab_room_full_tables
```

Now, in one terminal, run:

```bash
roslaunch node_a launcher1.launch
```

Open another terminal, run
```bash 
roslaunch node_a launcher2.launch
```
