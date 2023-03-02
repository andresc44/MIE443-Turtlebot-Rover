# MIE443-Turtlebot-Rover
## Drive link https://drive.google.com/drive/folders/1YfIvuKWdbFePLj8CiYQTBzpMpnKyqV45

## Steps for running code for contest 1
Assuming that the host computer already has a catkin_ws and the required packages installed
`cd catkin_ws`<br />
`cd src`<br />

Insert unzipped package into this location<br />
`cd ..` to the catkin_ws directory<br />
`source devel/setup.bash`<br />
`catkin_make`<br />


`roslaunch turtlebot_bringup minimal.launch`<br />
`roslaunch mie443_contest1 gmapping.launch`<br />
`roslaunch turtlebot_rviz_launchers view_navigation'launch`<br />

At this point, maximize the RViz window<br />
Open the terminal window at a quarter of the screen size, and overlay on the top left corner of the RViz window<br />
`rosrun mie443_contest1 contest1`<br />

The robot should begin traversing maze. Starts by moving forward unless there is an obstacle directly in front of it<br />

Once the robot has finished traversing maze, it is still reading scan data to create the map<br />

**DO NOT WALK IN FRONT OF THE ROBOT AT THIS POINT**

In the window where the contest 1 node was run, enter the following:<br />

`rosrun map_server map_saver -f contest1_map`<br />

Confirm that the map was saved in the current directory<br />

Close all terminal windows
