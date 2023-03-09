// Function variables: variable_name
// Functions: functionName
// Definitions: DEFINITION_NAME
// Global variables: VariableName

// 2 empty lines between functions
// 1 empty line after if, else if, else, for, and while (unless followed by another '}' )
// Initialization of variables at top in order of types
// Comments aligned on function level
// Comments start with space and capital

//CONVENTION
//CCW rotation is positive
//forward is x direction
//left is y direction

#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>

#include <stdio.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define LOOP_RATE 10                            // Rate for while loops that dictate callback and publish frequency (Hz)

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "cereal_recognition");
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);                             // Code will try to run at LOOP_RATE Hz

    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0); //index as robotPose.x, robotPose.y, robotPose.phi
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates

    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " phi: " 
                  << boxes.coords[i][2] << std::endl;
    }

    // Initialize image objectand subscriber.
    ImagePipeline imagePipeline(n);

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    uint8_t target = 255;
    bool visited[5] = {1, 1, 1, 1, 1};
    uint8_t results[5] = {0, 0, 0, 0, 0}; //1: Raisin Bran, 2: Cinnamon Toast Crunch, 3: Rice Krispies, 4: Blank
    uint8_t accum = 0;

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300 && (accum > 0)) {
        ros::spinOnce();
        target = GetNearestNeighbour(); //output uint8_t from 0-4, input is ??
        if (target > 4) continue;
        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = boxes.coords[target][0]// index coordinates file;
        goal.target_pose.pose.position.y = boxes.coords[target][1]// index coordinates file;
        goal.target_pose.pose.position.w = boxes.coords[target][2]// index coordinates file; //keep an eye out for yaw and w quaternion
        // test validity

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Arrived at target");
            //add while loop with timer for image recog
            while ((camera_timer < camera_limit) && !(isImageIdentified))
            label = imagePipeline.getTemplateID(boxes); //int from 1-4
            visited[target] = 0;
            results[target] = label;// OpenCV

            accum = accumulate(visited, visited + 5, accum=0);

            if (accum == 1) {
                if (CheckRepeat()) { //input is visited array, output is boolean
                    results = FillResults(results); // Input is results array, output is results array
                    break;
                }
            }

        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");

        loop_rate.sleep();                              // Delay function for 100ms

    }
    return 0;
}
