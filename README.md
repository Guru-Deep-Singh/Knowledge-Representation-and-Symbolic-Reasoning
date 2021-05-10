# Retail Store Stocking & 	Preparing Delivery Orders

This is a project made for the course RO470014 - Knowledge representation and decision making. The project deals with a retail store simulation environment in gazebo. which was provided by the course coordinator. The planning of the project is done using the robot Tiago. The implementation was done using PDDL 2.1 and  ROS Plan. The submission also includes the design of an ontology which was done in protege. The ontology was designed in order to first conceptualize our task and be ale to code them in PDDL. However, a future task could be to merge the ontology and the PDDL plan.

## Planning Task Description 
The robot is responsible for:

a) Preparing Delivery Orders
b) Stocking items on cabinets.
	
• Start Condition:
 - Milk (aruco_cube_444)  is on table 3
 - Yogurt is on table 2

•Goal Condition :
 - Stock milk on the cabinet (cabinet 2). 
 - Place Yogurt (aruco_cube_222) on  the Delivery Table (table 1).
 
<a href="https://imgbb.com/"><img src="https://i.ibb.co/QrrsvZx/Whats-App-Image-2021-04-05-at-17-57-55.jpg" alt="Whats-App-Image-2021-04-05-at-17-57-55" border="0" class="center"></a> 

## Run & Installation Process

In order to setup the repository follow the steps bellow from the directory with your singularity image:

1) Setting Up your workspace:
		
	    singularity shell -p ro47014-20-10-3.simg
	    source /opt/ros/melodic/setup.bash 
	    mkdir group16_workspace
	    cd  group16_workspace
	    mkdir src
	    cd src
	    git clone  https://github.com/KCL-Planning/ROSPlan.git
		git clone https://gitlab.tudelft.nl/cor/ro47014/2021_course_projects/group_16/retail_store_lightweight_sim_students.git

2) Add username and password
3) Build The repository

	    cd ..
	    catkin build


2) Open two more terminal windows :

	**Terminal 1**
	
		cd  group16_workspace
		source devel/setup.bash
		roslaunch retail_store_simulation tiago_simulation.launch world:=group16 use_aruco:=false

	**Terminal 2**
	
		singularity shell -p ro47014-20-10-3.simg
		source /opt/ros/melodic/setup.bash
		cd group16_workspace
		source devel/setup.bash
		roslaunch retail_store_planning rosplan_place.launch

	**Terminal 3**
		
		singularity shell -p ro47014-20-10-3.simg
		source /opt/ros/melodic/setup.bash
		cd  group16_workspace
		source devel/setup.bash
		cd src/retail_store_lightweight_sim_students/retail_store_planning/
		./rosplan_executor.bash

## Expected Simulation Result

The results of the simulations can be viewed in the following video:  [Simulation Results - YouTube](https://www.youtube.com/watch?v=bzXp75jHer8)

What you should see after running the simulation is:

1.  Robot moves from wp0 to table 3
2. Pick up the right object in front of it
3. Moves to cabinet 2 
4. Places the object on cabinet 2.
5. Robot moves to table 2 
6. Picks up the object on its right
7. Moves to table 1
8. Places the object on table 1

<a href="https://imgbb.com/"><img src="https://i.ibb.co/NVpXNtm/Whats-App-Image-2021-04-05-at-18-16-06.jpg" alt="Whats-App-Image-2021-04-05-at-18-16-06" border="0"></a>


## Folders Explanation

**Important Directories:**

**[Submission_OWL_Ontology](https://gitlab.tudelft.nl/cor/ro47014/2021_course_projects/group_16/retail_store_lightweight_sim_students/-/tree/master/Submission_OWL_Ontology)**  :  Contains the OWL ontology which was designed in protege. This ontology at a later stage can be integrated with the PDDL plan.



**[retail_store_planning](https://gitlab.tudelft.nl/cor/ro47014/2021_course_projects/group_16/retail_store_lightweight_sim_students/-/tree/master/Submission/retail_store_lightweight_sim_students/retail_store_planning)** : Contains the files required for planning the motion of the Tiago robot.
 - [pddl_files](https://gitlab.tudelft.nl/cor/ro47014/2021_course_projects/group_16/retail_store_lightweight_sim_students/-/tree/master/retail_store_planning/pddl_files): Includes the PDDL files used for planning.
     - `domain_pick.pddl` : Domain file used for Picking  from table
	 - `problem_pick.pddl`: Problem file used for Picking  from table
	 - `domain_place.pddl` : Domain file used for Picking and Placing objects from tables and cabinets.
	 - `problem_place.pddl`:  Problem file used for Picking and Placing objects from tables and cabinets.
	 - `domain_move.pddl` : Domain file used for moving to all the tables.
	 - `problem_move.pddl`:  Problem file used for moving to all the tables inside the simulation.
	
 - [src](https://gitlab.tudelft.nl/cor/ro47014/2021_course_projects/group_16/retail_store_lightweight_sim_students/-/tree/master/retail_store_planning/src):  Contains the C++ Nodes responsible for pick and place action.
	 - `RPPick.cpp`:   Node which is launched while performing the pick action
	 - `RPPlace.cpp:` Node which is launched while performing the place action




**[retail_store_simulation](https://gitlab.tudelft.nl/cor/ro47014/2021_course_projects/group_16/retail_store_lightweight_sim_students/-/tree/master/Submission/retail_store_lightweight_sim_students/retail_store_simulation)**: This directory contains the launch files for running the simulation as well as different versions of the simulation world.

**[tiago_example_skills](https://gitlab.tudelft.nl/cor/ro47014/2021_course_projects/group_16/retail_store_lightweight_sim_students/-/tree/master/Submission/retail_store_lightweight_sim_students/tiago_example_skills)**: Contains the elementary actions that the Tiago robot can perform. This includes: 

 - Moving its base from one way-point to another (`move_base.py`)  
 - Gripper Control used for opening or closing the gripper of Tiago   
   (`gripper_control.py`) .
  - Look to point used to direct the head of  
   Tiago to a specific point (`gripper_control.py`) . 
   - Used to enable   Tiago picking an object (`pick_server.py`) . 
   - Used to enable Tiago placing an object (`place_server.py`) . 
   - Used for planning the    Inverse Kinematics of Tiago's Arm (`place_server.py`) .

