# ltl_automaton_6dof_hebi_arm
Provides integration of the 6-DoF Hebi manipulator with LTL automaton package

## Installation

### Dependencies
- [ltl_automaton_core](https://github.com/KTH-SML/ltl_automaton_core). Core ROS package for LTL.

- [HEBI-ROS](http://wiki.ros.org/hebiros). HEBI Robotics ROS package.
  - For ROS Kinetic, install using `sudo apt install ros-kinetic-hebiros`
  - For ROS Melodic, Noetic or more recent, build the following packages from source:
    - [https://github.com/HebiRobotics/HEBI-ROS-DEPRECATED](https://github.com/HebiRobotics/HEBI-ROS-DEPRECATED)

- [PyYAML](https://pyyaml.org/). Should be integrated with ROS but it's better to check if version is up-to-date.
	- For Python2 (ROS Kinetic & Melodic):
	`pip install pyyaml`
	- For Python3 (ROS Morenia):
	`pip3 install pyyaml`
  
### Building
To build the package, clone the current repository in your catkin workspace and build it.
```
cd catkin_ws/src
git clone https://github.com/KTH-SML/ltl_automaton_6dof_hebi_arm
```
Build your workspace with either *catkin_make* or *catkin build*
```
cd ...
catkin_make
```

## Usage
The package provides the agent-level code needed for interacting with the 6-DoF Hebi manipulator.

To launch the planner and LTL Hebi arm node, simply run the following command.

```
roslaunch ltl_automaton_turtlebot ltl_6dof_hebi.launch
```

This will run the planner using the task specification in `config/6dof_hebi_ltl_formula.yaml` and transition system in `config/6dof_hebi_ts.yaml`.

### Transition system and actions
The robot transition system needs to be of the following type: `[6d_jointspace_region", "hebi_load]`. The robot can carry the following actions:
- `goto_<region>` (from *6d_jointspace_region*): Go to the defined regions. The action needs to be part of the transition system textfile with the following attributes
  
  ```Python
  attr:
  	jointposition: [0.1, 1.1, 2.1, 2.8, -1.5, 0.4] # Joint values for the 6 joints, following hebiros group name order
  ```
  
- `pick` (from *hebi_load*): Perform pick up action. The manipulator will wait for one of the agent defined in `pick_agent_list` to be on the region defined in `pick_region` and have in its TS state one of the state from `pick_agent_load_state_guard` before moving. Confirmation of finished action is sent on `pick_inplace_ack` topic.

- `drop` (from *hebi_load*): Perform drop action. The manipulator will wait for one of the agent defined in `drop_agent_list` to be on the region defined in `drop_region` and have in its TS state `unloaded` before moving. Confirmation of finished action is sent on `drop_inplace_ack` topic.





  `agent_name`  Default: "kth_hebi_arm"
  `initial_ts_state_from_agent` Default: "False"
  `pick_agent_list` Default: "[nexus]"
  `pick_agent_load_state_guard` Default: "[loaded]"
  `pick_region` Default: "s2"
  `drop_agent_list` Default: "[turtlebot]"
  `drop_region` Default: "s2"/
