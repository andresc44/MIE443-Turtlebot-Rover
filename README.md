# MIE443-Turtlebot-Rover
## Drive link https://drive.google.com/drive/folders/1YfIvuKWdbFePLj8CiYQTBzpMpnKyqV45

## Steps for running code for contest 2
Assuming that the host computer already has a catkin_ws and the required packages installed
`cd catkin_ws`<br />
`cd src`<br />

Insert unzipped package into this location<br />
`cd ..` to the catkin_ws directory<br />
`source devel/setup.bash`<br />
`catkin_make`<br />


`roslaunch turtlebot_bringup minimal.launch`<br />
`roslaunch turtlebot_navigation amcl_demo.launch map_file:=/<map_path>/my_map.yaml`<br />
`roslaunch turtlebot_rviz_launchers view_navigation.launch`<br />

At this point, maximize the RViz window<br />
Open the terminal window at a quarter of the screen size, and overlay on the top left corner of the RViz window<br />
`rosrun mie443_contest2 contest2`<br />

Once the robot has finished traversing maze, it will return to the starting location<br />

Close all terminal windows