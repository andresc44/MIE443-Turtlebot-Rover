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

int main(int argc, char** argv) {
    // Setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;
    // Robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robotPose);
    // Initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
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

    // Execute strategy.
    while(ros::ok() && secondsElapsed <= 300 && (accum > 0)) {
        ros::spinOnce();
        target = GetNearestNeighbour();
        //we'll send a goal to the robot to move 1 meter forward
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = // index coordinates file;
        goal.target_pose.pose.position.y = // index coordinates file;
        goal.target_pose.pose.position.w = // index coordinates file; //keep an eye out for yaw and w quaternion
        // test validity

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
            /***YOUR CODE HERE***/
            // Use: boxes.coords
            // Use: robotPose.x, robotPose.y, robotPose.phi
            label = imagePipeline.getTemplateID(boxes);
            visited[target] = 0;
            results[target] = label;// OpenCV
            accum = accumulate(Visited);
            if (accum == 1) {
                if (CheckRepeat()) {
                    FillResults();
                    break;
                }
            }

        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");

        ros::Duration(0.01).sleep();
    }
    return 0;
}
