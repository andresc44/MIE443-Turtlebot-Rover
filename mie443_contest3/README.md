# MIE443-Turtlebot-Rover Group 7
## Drive link https://drive.google.com/drive/folders/1YfIvuKWdbFePLj8CiYQTBzpMpnKyqV45

## Steps for running code for contest 3
Assuming that the host computer already has a catkin_ws and the required packages installed
`cd catkin_ws`<br />
`cd src`<br />

Insert unzipped package into this location<br />
`cd ..` to the catkin_ws directory<br />
`source devel/setup.bash`<br />
`catkin_make`<br />


`roslaunch turtlebot_bringup minimal.launch`<br />
`roslaunch turtlebot_follower follower.launch`<br />
`rosrun sound_play soundplay_node.py`<br />

`rosrun mie443_contest3 contest3`<br />

Once the timer has expired, the robot will remain stationary<br />

Close all terminal windows