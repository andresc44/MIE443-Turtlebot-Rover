# MIE443-Turtlebot-Rover

For the Lenova PC, go to the MIE443-Turtlebot-Rover directory, enter: 
`chmod +x modified_turtlebot_script.sh`
`sh modified_turtlebot_script.sh`

The installation should happen

For anyone else:
`chmod +x turtlebot_script.sh`
`sh turtlebot_script.sh`

This file will install ROS, turtlebot, VSCode, and Terminator

`source devel/setup.bash`

for Andres wifi:

`sudo systemctl restart network-manager`

Launch packages: 
`roslaunch mie443_contest1 gmapping.launch`

`roslaunch mie443_contest1 turtlebot_world.launch`