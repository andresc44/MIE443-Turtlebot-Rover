// Remember to check: gazebo launch file directory, image topic, which launch file to use, thresholds, and definitions, boxes.cpp file
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!SET MAX TIMER ON WHILE LOOP TO 300 SECONDS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define MAX_CONTEST_TIME 3000 //change to 300
#include <iostream>
#include <string>
#include <fstream>
std::string result_path ("/home/andres/turtlebot-ws/src/MIE443-Turtlebot-Rover/mie443_contest2/src/results.txt");
// MIE443 CONTEST 2
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

// template id 1 = kelloggs raisin bran
// template id 2 = cinnamon toast crunch
// template id 3 = kelloggs rice krispies

//!!!!!!!LIBRARIES!!!!!!!!!!!!!!!!!!!!!
#include <boxes.h>
#include <navigation.h>
#include <robot_pose.h>
#include <imagePipeline.h>
#include <chrono>

#include <stdio.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cmath>
#include <math.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>


//!!!!!!CONSTANTS!!!!!!!!!!!!!!!!!!!!!
#define LOOP_RATE 10                                // Rate for while loops that dictate callback and publish frequency (Hz)
#define STARTING_VISION_RADIUS 0.5     //Try to bump to 40   // The distance from the centre of the turtlebot to the front of the cereal box when parked and scanning
#define MAX_VISION_RADIUS 0.7
#define RADIUS_JUMP 0.05
#define ANGLE_INCREMENT 10 //to be changed          // Degrees by which to increase in search for optimal goal destination from the centre
#define MAX_ANGLE 60       //to be changed          // Max possible angle to deviate from centre optimal spot
#define CAMERA_LIMIT 10                             // Seconds to be stuck trying to read an image
#define ARRIVAL_DELAY 1                             // Time to wait before starting to try to scan camera [s]
#define GOAL_TOLERANCE 0.03                         // Tolerance for goal destination when trying to make plan (x, and y)
#define FORWARD_JUMP_SIZE 0.25                       // Move forward by 0.2m

// !!!!!!!!!!GLOBAL VARIABLES!!!!!!!!!!!!!!!!!!!
ros::Publisher VelPub;
geometry_msgs::Twist Vel; 

// !!!!!!!!!!!!USER-DEFINED FUNCTION!!!!!!!!!!


int getNearestNeighbour(RobotPose robot_pose, std::vector<std::vector<float>> box_coordinates, bool visited[5]){
    // 	Inputs:
    //	robot_pose = (x,y,phi)
    //	box_coordinates = box.coords
    //	visited = [1,1,1,1,1]
    //	Outputs:
    //	closest_idx = index of closest, unvisited box
    

    //  compare box_coords array with visited array
    int sum = std::accumulate(visited, visited+5, sum=0);  // Length of new list of possible targets
    std::vector<std::vector<float>> possible_target{sum, std::vector<float>(3)};		// Initialize empty list of possible targets
    int possible_idx = 0;

    while (possible_idx < sum){
        for (int i = 0; i < box_coordinates.size(); ++i){
            if (visited[i] == 1){ 				// Means location is unvisited
                for (int j = 0; j < 3; j++){
                    possible_target[possible_idx][j]= box_coordinates[i][j]; // Add unvisited coordinates to possible_target vector
                    // std::cout << i << j << "Possible: " << possible_target[possible_idx][j] << std::endl;
                }
                possible_idx += 1;
            }
        }
    }
    
    

    // Find closest index 
    float closest_dist = 1000.0;
    int closest_idx_p = -1;  // Index of cloest coordinates in the possible_target array
    int closest_idx = -1;  // Index of closest coordinates in the boxes_coordinates (AKA true) array
    
    for(int i = 0; i < sum; ++i) {
        float x = possible_target[i][0];
        float y = possible_target[i][1];
        // std::cout << i << "x distance: " << x << "y distance: " << y << std::endl;

        float dx = x - robot_pose.x;
        float dy = y - robot_pose.y;
        float distance = std::sqrt(dx*dx+dy*dy);
        // ROS_INFO("Possible Array Index: %i", i);
        // ROS_INFO("Distance: %f", distance);
        if (distance < closest_dist){
            closest_dist = distance;
            closest_idx_p = i;
            // ROS_INFO("Closest_idx_p: %i", closest_idx_p);
        }
    }
    for (int i = 0; i < box_coordinates.size(); ++i) {
        bool same = false;
        for (int j = 0; j < 3; j++) {
            if (box_coordinates[i][j] == possible_target[closest_idx_p][j]){
                // ROS_INFO("Equal %i", j);
                same = true;
            }
            else{
                // ROS_INFO("Not Equal %i, %i", i, j);
                same = false;
            }
        }
        if (same){
            closest_idx = i;
        }
    }
    return closest_idx;
}


// In case std::accumulate doesn't work...
int sum(int array[5]){
    int sum = 0;
    for(int i = 0; i <5; i++){
        sum += array[i];
    }
    return sum;
}


int checkRepeat(int results_array[5]){
    for (int i = 0; i<5; i++){
        for (int n = i+1; n <5; n++){
            if ((results_array[n] == results_array[i]) && (results_array[i] != -1) && (results_array[i] != 3) && (results_array[i] != 4)){ //!!!!!!!NEW ADDITION
                return true;
            }
        }
    }
    return false;
}


// Duplicate =  True and therefore the last unvisited location is unique (not a repeat)
std::tuple<int, int> fillResults(int results_array[5]){
    int unvisited_idx = -1;
    int missing_id = -1;
    bool cases[4] = {0,0,0,0};

    for (int i = 0; i<5; i++){
        if (results_array[i] == 0){ // matches with raisin
            // ROS_INFO("There is Raisin");
            cases[0] = 1;
        }
        else if (results_array[i] == 1){ // matches with cinnamon
            cases[1] = 1;
            // ROS_INFO("There is Cinnamon");
        }
        else if (results_array[i] == 2){ // matches with rice
            cases[2] = 1;
            // ROS_INFO("There is Rice");
        }
        else if (results_array[i] ==3){ // matches with blank
            cases[3] = 1;
            // ROS_INFO("There is blank");
        }
        else{ // no match then it's unvisited = 55
            unvisited_idx = i;
            // ROS_INFO("Unvisited Index: %i", unvisited_idx);
        }
    }
    for (int i = 0; i<4; i++){
        // std::cout << "Case: " << i << "is " << cases[i] << std::endl;
        if (cases[i] == false){
            // ROS_INFO("Case is false");
            missing_id = i;
        }
    }
    // ROS_INFO("Missing Id: %i", missing_id);
    return std::tuple<int, int> {unvisited_idx, missing_id};
}

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                                      // Create message for velocities as Twist type
int linear_move(ros::Publisher VelPub, float linear_vel, float distance, int direction)
{
    // if (linear_vel == float(FORWARD_FAST_V)) ROS_INFO("forward fast!");
    // else if (linear_vel == float(FORWARD_SLOW_V)) ROS_INFO("forward slow");
    // else ROS_INFO("Forward but unsure why, linear_vel is: %f", linear_vel);
    ros::Rate loop_rate(LOOP_RATE);
    std::chrono::time_point<std::chrono::system_clock> start;       // Initialize timer

    Vel.angular.z = 0.0;
    Vel.linear.x = linear_vel*direction;
    uint64_t secondsElapsed = 0;                                    // Variable for time that has passed
    float time_forward = abs(distance/linear_vel);

    start = std::chrono::system_clock::now();                       // Start the timer

    while (ros::ok() && secondsElapsed <= time_forward) {
        VelPub.publish(Vel);
        ros::spinOnce();                                            // Listen to all subscriptions once
        // if (AnyBumperPressed) {
        //     break;
        // }
        loop_rate.sleep();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
    }
}

int getTrueLabel(int labels_views[3]) {
    int true_label = 255;
    if ((labels_views[0] == labels_views[1]) && (labels_views[1] == labels_views[2])) true_label = labels_views[0];

    else if ((labels_views[0] == labels_views[1])) true_label = labels_views[1];

    else if ((labels_views[1] == labels_views[2])) true_label = labels_views[1];

    else if ((labels_views[0] == labels_views[2])) true_label = labels_views[0];

    else true_label = labels_views[1];
    
    return true_label;
}



// !!!!!!!!!!!!!MAIN!!!!!!!!!!!!!!!!!!!!!!!!!!
int main(int argc, char** argv) {
    ros::init(argc, argv, "cereal_recognition");    // Initialize node name
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);                 // Code will try to run at LOOP_RATE Hz

    RobotPose robot_pose(0,0,0);                     // Index as robot_pose.x, robot_pose.y, robot_pose.phi
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robot_pose);
    ros::ServiceClient move_client = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan"); // Client for the make_plan service
    
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    VelPub = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); // Publish cmd_velocity as Twist message
    
    Boxes boxes;                                    // Initialize box coordinates and templates
    if(!boxes.load_coords() || !boxes.load_templates()) {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i) {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " phi: " 
                  << boxes.coords[i][2] << std::endl;
    }


    ImagePipeline imagePipeline(n);                 // Initialize image object and subscriber.


    std::chrono::time_point<std::chrono::system_clock> start; // Contest count down timer
    start = std::chrono::system_clock::now();

    std::chrono::time_point<std::chrono::system_clock> camera_start; // Timer for camera to get result
    
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::Quaternion q;
    nav_msgs::GetPlan srv;
    

    uint64_t camera_seconds_elapsed = 0;
    uint64_t seconds_elapsed = 0;
    uint8_t loop_iterations = 0;
    int temp_labels[3] = {255, 255, 255};
    int results[5] = {255, 255, 255, 255, 255};           // 0: Raisin Bran, 1: Cinnamon Toast Crunch, 2: Rice Krispies, 3: Blank
    // int results[5] = {0, 3, 1, 2, 255};           // 0: Raisin Bran, 1: Cinnamon Toast Crunch, 2: Rice Krispies, 3: Blank
    int accum = 255;
    int target = 255;
    int skipper = 0;
    int label = 255;
    int label_iter = 0;
    int view_iter = 0;
    int view = 0;
    int i = 0;
    float offset_angle = 0;
    float yaw_adjust = 0;
    float trajectory_x = 0;
    float trajectory_y = 0;
    float trajectory_phi = 0;
    float cereal_x = 0;
    float cereal_y = 0;
    float cereal_yaw = 0;
    float initial_x = 0;
    float initial_y = 0;
    float initial_phi = 0;
    bool visited[5] = {1, 1, 1, 1, 1};
    bool successful_plan = false;
    bool is_positive = 0;
    bool is_back_at_start = false;
    bool arrived_at_target = 0;

    uint8_t missing_id = 255;
    uint8_t missing_label = 255;

    

    

    // Execute strategy.
    while(ros::ok() && seconds_elapsed <= MAX_CONTEST_TIME && (!is_back_at_start)) {
        ros::spinOnce();
        loop_iterations += 1;
        if (loop_iterations < 5) { //Buffer so that robot pose can get accurate origin location rather than assuming zero
            ros::spinOnce();
            initial_x = robot_pose.x;
            initial_y = robot_pose.y;
            initial_phi = robot_pose.phi;
            loop_rate.sleep();
            continue;
        }

        if (accum > 0) {
            // ROS_INFO("Robot Pose: (%f, %f, %f)", robot_pose.x, robot_pose.y, robot_pose.phi);
            target = getNearestNeighbour(robot_pose, boxes.coords, visited); //output uint8_t from 0-4
            ROS_INFO("target: %i ", target);
            if (target > 4) continue;

            // // bool x = 0;
            // // x = checkRepeat(results);
            // // ROS_INFO("checkRepeat: %i", x);
            // // std::tie(missing_id, missing_label) = fillResults(results);
            // // ROS_INFO("Missing_id should be 1 but is %i and label should be 2 but is %i", missing_id, missing_label);

            cereal_x = boxes.coords[target][0];
            cereal_y = boxes.coords[target][1];
            cereal_yaw = boxes.coords[target][2];

            // ROS_INFO("target x: %f, target y: %f, target yaw: %f", cereal_x, cereal_y, cereal_yaw);


            successful_plan = false;

            
            // Looping different STARTING_VISION_RADIUS values. If successful plan, exit loop
            for (float radius = STARTING_VISION_RADIUS; radius <= MAX_VISION_RADIUS; radius += RADIUS_JUMP) { // Try at different radius values
                ROS_INFO("Attempting radius: %f, view: %i", radius, view);
                switch(view) {
                    case 0:
                        i = 5; ////normal attempt fron front ideally
                        skipper = 2; // Only increase i to get odd numbers, starting at 30 degrees positive
                        break;
                    case 1:
                        i = 0;       //increase i by 1 as normal 
                        skipper = 1;
                        break;
                    case 2:
                        i = 6;      // Only increase i to get odd numbers, starting at 30 degrees negative
                        skipper = 2;
                        break;
                }
                // i = 0;
                offset_angle = 0;
                while (offset_angle <= MAX_ANGLE) {
                    // ROS_INFO("i = %i", i);
                    is_positive = i % 2;
                    if (is_positive) {                                      // Odd number for i, positive angle offset
                        offset_angle = ANGLE_INCREMENT * (i + 1)/2;         // Increment of ANGLE_INCREMENT degrees
                        ROS_INFO("Offset_angle: %f", offset_angle);
                        yaw_adjust = (offset_angle * M_PI/180) + cereal_yaw;
                    } 
                    
                    else { //even numbers, include the initial 0 and the sunsequent clockwise locations in negative direction
                        offset_angle = ANGLE_INCREMENT * i/2;
                        ROS_INFO("Offset_angle: %f", -offset_angle);
                        yaw_adjust = -(offset_angle * M_PI/180) + cereal_yaw;
                    }

                    // ROS_INFO("yaw_adjust radians: %f, degrees: %f", yaw_adjust, yaw_adjust*180/M_PI);
                    trajectory_x = radius*cosf(yaw_adjust) + cereal_x;
                    trajectory_y = radius*sinf(yaw_adjust) + cereal_y;
                    trajectory_phi = (yaw_adjust + M_PI); //!!!!!!!Check what the bounds of this need to be
                    // ROS_INFO("trajectory x: %f, trajectory y: %f, Trajectory_phi: %f", trajectory_x, trajectory_y, trajectory_phi);
                    // // try to make plan with trajectory_x, trajectory_y, trajectory_phi, if so, break loop
                    target_pose.header.stamp = ros::Time::now();
                    target_pose.header.frame_id = "map";
                    target_pose.pose.position.x = trajectory_x; // index coordinates file;
                    target_pose.pose.position.y = trajectory_y; // index coordinates file;

                    q = tf::createQuaternionMsgFromYaw(trajectory_phi);
                    target_pose.pose.orientation.x = 0.0;
                    target_pose.pose.orientation.y = 0.0;
                    target_pose.pose.orientation.z = q.z;
                    target_pose.pose.orientation.w = q.w;
                    // ROS_INFO("q.z: %f, q.w: %f", q.z, q.w);

                    start_pose.header.stamp = ros::Time::now();
                    start_pose.pose.position.x = robot_pose.x; // index coordinates file;
                    start_pose.pose.position.y = robot_pose.y; // index coordinates file;

                    q = tf::createQuaternionMsgFromYaw(robot_pose.phi);
                    start_pose.pose.orientation.x = 0.0;
                    start_pose.pose.orientation.y = 0.0;
                    start_pose.pose.orientation.z = q.z;
                    start_pose.pose.orientation.w = q.w;

                    srv.request.start  = start_pose;
                    srv.request.goal = target_pose;
                    srv.request.tolerance = GOAL_TOLERANCE;                   // Tolerance of destination goal in [m]


                    move_client.call(srv);
                    // ROS_INFO("srv.response.plan.poses.size: %i", srv.response.plan.poses.size());
                    if (srv.response.plan.poses.size() > 0) {
                        ROS_INFO("Successful plan found at radius %f, angle: %f, counter-clockwise: %i", radius, offset_angle, is_positive);
                        successful_plan = true;
                        break;
                    }
                    i+=skipper;
                }
                if (successful_plan) break;
            }
        }
        else { // All labels at locations are known
            is_back_at_start = true;
            ROS_INFO("Returning to start, no locations left");
            Navigation::moveToGoal(initial_x, initial_y, initial_phi);
            ROS_INFO("Final results: [%i, %i, %i, %i, %i]", results[0], results[1], results[2], results[3], results[4]);
        }
        // ROS_INFO("TARGETS x: %f, y: %f, phi: %f, offset angle: %f", trajectory_x, trajectory_y, trajectory_phi, offset_angle);
        // arrived_at_target = Navigation::moveToGoal(-1.182420, 1.626125, 2.535593); // Location 0
        // arrived_at_target = Navigation::moveToGoal(-1.182420, 1.45, 2.535593); // Location 0 alternative, due to non-perfect map at edges
        // arrived_at_target = Navigation::moveToGoal(0.183988, 0.9923, 3.132593); // Location 1
        // arrived_at_target = Navigation::moveToGoal(-0.107461, -0.70500, 1.572593); // Location 2
        // arrived_at_target = Navigation::moveToGoal(2.280940, -1.036683, 3.932593); // Location 3
        // arrived_at_target = Navigation::moveToGoal(1.887739, 1.743003, 1.566593); // Location 4
        // arrived_at_target = Navigation::moveToGoal(1.887739, 1.653003, 1.566593); // Location 4 alternative
        if (successful_plan && !is_back_at_start) arrived_at_target = Navigation::moveToGoal(trajectory_x, trajectory_y, trajectory_phi);

        else if (!is_back_at_start){ //skip this destination rather than crash the move_base
            temp_labels[view] = -1;
            ROS_INFO("Failed to create successful plan to navigate to target: %i in time with view: %i", target, view);
            if (view == 2) {

                visited[target] = 0;
                results[target] = getTrueLabel(temp_labels);
                accum = std::accumulate(visited, visited + 5, accum=0);
                ROS_INFO("Intermediate results: [%i, %i, %i, %i, %i]", results[0], results[1], results[2], results[3], results[4]);
                ROS_INFO("Intermediate visited: [%i, %i, %i, %i, %i]", visited[0], visited[1], visited[2], visited[3], visited[4]);
                ROS_INFO("Locations left to visit: %i", accum);
                if (accum == 1) {
                    if (checkRepeat(results)) { //input is visited array, output is boolean
                    // if (true) { //input is visited array, output is boolean
                        ROS_INFO("There are repeated labels, can skip the last destination");
                        std::tie(missing_id, missing_label) = fillResults(results); // Input is results array, output is results array
                        results[missing_id] = missing_label; 
                        ROS_INFO("Filled in results: [%i, %i, %i, %i, %i]", results[0], results[1], results[2], results[3], results[4]);
                        accum = 0;
                    }
                    else {
                        ROS_INFO("There are no repeats, or results contains error value/non-deterministic [-1, 3, 4]");
                    }
                }
                
            }
            view_iter++;
            view = view_iter % 3;
            continue;
        }

        if((arrived_at_target) && (!is_back_at_start)) {
            // ros::Duration(ARRIVAL_DELAY).sleep();               // Small pause to account for momentum shift
            if (view == 1) linear_move(VelPub, 0.1, FORWARD_JUMP_SIZE, 1); //0.1m/s, 0.2m, forward
            else linear_move(VelPub, 0.1, FORWARD_JUMP_SIZE/2, 1); //0.1m/s, 0.2m, forward
            // ros::spinOnce();
            ros::Duration(ARRIVAL_DELAY).sleep();
            camera_start = std::chrono::system_clock::now();
            camera_seconds_elapsed = 0;
            label = -1;
            ROS_INFO("Waiting for valid image pipeline output");
            
            int idArray[5];
            while ((camera_seconds_elapsed < CAMERA_LIMIT) && (label == -1)) {
                ros::spinOnce();
               
                // for (int k=0; k<5; k++)
                // {
                //     ros::spinOnce();
                //     label_iter = imagePipeline.getTemplateID(boxes); //int from 0-3
                //     // label = 4;
                //     idArray[k]=label_iter;
                //     loop_rate.sleep();
                // }

                // label = 4;
                label = imagePipeline.getTemplateID(boxes); //int from 0-3
                camera_seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-camera_start).count(); //time elapsed trying to read image
                loop_rate.sleep(); 
            }

            // int label = idArray[0];
            // for (int s=0; s < 5; s++)
            // {
            //     if (idArray[s] < label)
            //     {
            //         label = idArray[s];
            //     }
            // }

            
            ROS_INFO("label: %i at location: %i, view: %i", label, target, view);
            temp_labels[view] = label;

            if (view == 2) {
                label = getTrueLabel(temp_labels);
                visited[target] = 0;
                results[target] = label;
                ROS_INFO("Intermediate results: [%i, %i, %i, %i, %i]", results[0], results[1], results[2], results[3], results[4]);
                ROS_INFO("Intermediate visited: [%i, %i, %i, %i, %i]", visited[0], visited[1], visited[2], visited[3], visited[4]);
                accum = std::accumulate(visited, visited + 5, accum=0);
                ROS_INFO("Locations left to visit: %i", accum);
                if (accum == 1) {
                    if (checkRepeat(results)) { //input is visited array, output is boolean
                    // if (true) { //input is visited array, output is boolean
                        ROS_INFO("There are repeated labels, can skip the last destination");
                        std::tie(missing_id, missing_label) = fillResults(results); // Input is results array, output is results array
                        results[missing_id] = missing_label; 
                        ROS_INFO("Filled in results: [%i, %i, %i, %i, %i]", results[0], results[1], results[2], results[3], results[4]);
                        accum = 0;
                    }
                    else {
                        ROS_INFO("There are no repeats, or results contains error value/non-deterministic [-1, 3, 4]");
                    }
                }
            }
            if (view == 1) linear_move(VelPub, 0.1, FORWARD_JUMP_SIZE, -1); //0.1m/s, 0.2m, backward
            else linear_move(VelPub, 0.1, FORWARD_JUMP_SIZE, -1); //0.1m/s, 0.2m, backward
            view_iter++;
            view = view_iter % 3;
        }
        else {
            ROS_INFO("Plan was successful but failed to arrive, likely because too close to wall. Reversing now");
            linear_move(VelPub, 0.1, FORWARD_JUMP_SIZE/2, -1); //0.1m/s, 0.2m, backward, get unstuck
        }
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); // Count how much time has passed
        loop_rate.sleep();                              // Delay function for 100ms

    }
    std::ofstream file;
    file.open(result_path);
    file << "label at location 0: " << results[0] << std::endl << "label at location 1: " << results[1] << std::endl << "label at location 2: " << results[2] << std::endl << "label at location 3: " << results[3] << std::endl << "label at location 4: " << results[4] << std::endl;
    ROS_INFO("Finished saving results array to text file");
    return 0;
}
