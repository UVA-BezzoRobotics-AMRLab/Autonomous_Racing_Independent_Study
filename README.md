# Autonomous_Racing_Independent_Study
This project creates an autonomous Jackal robot that can avoid moving obstacles. Using this algorithm, this repository provides code for 3 scenarios:
1. Navigating around a highway with many moving obstacles
2. Racing against a manually controlled robot in a highway
3. Racing against a manually controlled robot in a racetrack

## Setup
Need ROS Kinetic, Gazebo, catkin_pkg, and ros keyboard

Setup the catkin_ws folder (needs to be done every time the code in catkin_ws is changed)

`cd catkin_ws`

`catkin_make`

Setup the jackal_ws folder (needs to be done every time the code in jackal_ws is changed)

`cd ../jackal_ws`

`catkin_make`

Source the setup file

`cd ../`

`source catkin_ws/devel/setup.bash`

`source jackal_ws/devel/setup.bash`

The source commands need to be done for every new terminal. To avoid this, .bashrc can be modified to include these source commands

## Scenario 1: Highway with Many Obstacles

Launch the launch file

`roslaunch multi_jackal_tutorials jackal_highway_long_6obs.launch`

Open Gazebo

`gzclient`

Run the code

`rosrun jackal_nodes particle_planner_highway`

This runs the highway scenario with 6 obstacles. To test the scenario with 12 obstacles, run `roslaunch multi_jackal_tutorials jackal_highway_long_12obs.launch` instead.

Additionally, edit `particle_planner_highway.cpp` line 65 to be 12. Also uncomment lines 79 and 80 while commenting lines 82 and 83. After code changes, `catkin_make` is needed.

## Scenario 2: Highway Race

Launch the launch file

`roslaunch multi_jackal_tutorials jackal_highway_long_6obs.launch`

Open Gazebo

`gzclient`

Run the code

`rosrun keyboard keyboard`

`rosrun jackal_nodes jackal_teleop_racing`

`rosrun jackal_nodes particle_planner_highway_race`


## Scenario 3: Racetrack Race

Launch the launch file

`roslaunch multi_jackal_tutorials jackal_highway_long_6obs.launch`

Open Gazebo

`gzclient`

Run the code

`rosrun keyboard keyboard`

`rosrun jackal_nodes jackal_teleop_racing`

`rosrun jackal_nodes particle_planner_racetrack`

## Visualization
### Matlab Highway Visualization
`planner_visualization_v2.m` is a matlab visualization that shows the trajectories that the planner examines during the highway scenario. This visualizer only looks at the very first plan that is called when all the objects are at there starting positions. It uses recorded ros bag files.

To record another ros scenario, edit `particle_planner_highway.cpp` line 652 to be `if(true)` to stop the obstacles from physically moving. (This may not be needed, but this is what was done when testing the visualizer)

Then `catkin_make` and run the commands for scenario 1. Then go into the bagfiles folder and run 

`rosbag -a`

After a second, all the needed data will be recorded and the newly created bag file can be used by the visualizer.

### Creating ractrack waypoints and visualization
The list of waypoints was created by running Scenario 3 and running `jackal_teleop_racing`. The manually controlled car was driven from the starting point around the track and its positions were recorded in a file. To do this, modify `jackal_teleop_racing` by uncommenting lines 45, 55, and 93. Then run drive the manual controlled car around the racetrack. A new text file will be made in the catkin_ws folder.  Use `plot_track.py` to visualize the waypoints created.

## Credits and Email
jackal_ws was taken from https://github.com/NicksSimulationsROS/multi_jackal

Email nikileshsub@gmail.com for any questions or corrections needed
