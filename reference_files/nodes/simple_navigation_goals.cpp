#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
// move base action accepts goals from clients and attempts to move robot to specified position in the world
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  //creates an action client called move_base. We'll use this client o
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up and ready to begin processing goals
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  //create a goal to send to move_base (using move_base_msgs data type)
  move_base_msgs::MoveBaseGoal goal; //i'm going to guess that goal is just an xy position array

  //we'll send a goal to the robot to move 1 meter forward IN THE BASE LINK COORDINATE FRAME

  //ANDRES CHANGED fram to map
  goal.target_pose.header.frame_id = "map"; // SPECIFY COORDINATE FRAME TO MOVE RELATIVE TO
  goal.target_pose.header.stamp = ros::Time::now();

  // FUTURE CODE: we will have 4 modes: approach loading zone, object search, object pickup, object drop off
  // mode 1: approach loading zone (turn on LED1)
    // won't require if statement, since it runs regardless
  // mode 2: object search (this mode will turn on once mode 1 has been reached) (turn on LED2)
    // will require some input from mode 1. probably a while loop 
  // mode 3: object pickup (this mode will continue to run until we get input from TOF sensor that object has been picked up) (turn on LED3)
    // will require while loop to continue running until TOF sensor gets desired delta
  // mode 4: object drop off (this will mode will turn on once we get the correct TOF sensor input) (turn on LED4)
    // move until end zone and drop object off 
  

  // // Dropoff Point 1
  // goal.target_pose.pose.position.x = 0.185; // x coordinate
  // goal.target_pose.pose.position.y = 1.526; // y coordinate
  // goal.target_pose.pose.orientation.z = 0; // z rotation in quaternion
  // goal.target_pose.pose.orientation.w = 1.0; // w rotation in quaternion

  // // Dropoff Point 2
  // goal.target_pose.pose.position.x = 0.795; // x coordinate
  // goal.target_pose.pose.position.y = 0.612; // y coordinate
  // goal.target_pose.pose.orientation.z = 0; // z rotation in quaternion
  // goal.target_pose.pose.orientation.w = 1.0; // w rotation in quaternion

  // Dropoff Point 3
  goal.target_pose.pose.position.x = 0.795; // x coordinate
  goal.target_pose.pose.position.y = 0.002; // y coordinate
  goal.target_pose.pose.orientation.z = 0; // z rotation in quaternion
  goal.target_pose.pose.orientation.w = 1.0; // w rotation in quaternion

  // // Dropoff Point 4
  // goal.target_pose.pose.position.x = 0.124; // x coordinate
  // goal.target_pose.pose.position.y = 0.002; // y coordinate
  // goal.target_pose.pose.orientation.z = 1.0; // z rotation in quaternion
  // goal.target_pose.pose.orientation.w = 0; // w rotation in quaternion

  // // Loading_Bottom
  // goal.target_pose.pose.position.x = 0.460; // x coordinate
  // goal.target_pose.pose.position.y = 2.136; // y coordinate
  // goal.target_pose.pose.orientation.z = 0; // z rotation in quaternion
  // goal.target_pose.pose.orientation.w = 1.0; // w rotation in quaternion

  // // Loading_Top
  // goal.target_pose.pose.position.x = 0.917; // x coordinate
  // goal.target_pose.pose.position.y = 1.679; // y coordinate
  // goal.target_pose.pose.orientation.z = 0.707; // z rotation in quaternion
  // goal.target_pose.pose.orientation.w = 0.707; // w rotation in quaternion

  ROS_INFO("Sending goal");
  ac.sendGoal(goal); // THIS ACTUALLY PUSHES THE GOAL MESSAGE TO THE MOVE_BASE NODE FOR PROCESSING


  ac.waitForResult(); // this command blocks everything else until the move_base action is done processing the goal

  //once goal is done processing we will check with if statement if the goal succeeded or failed and output a message acoordingly
  
  //if move_base succeeded then output "hooray"
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");

  //if move_base failed then output "sad ;("
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}