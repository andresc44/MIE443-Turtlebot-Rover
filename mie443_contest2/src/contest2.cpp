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

//!!!!!!CONSTANTS!!!!!!!!!!!!!!!!!!!!!
#define LOOP_RATE 10                                // Rate for while loops that dictate callback and publish frequency (Hz)
#define VISION_RADIUS 0.3                           // The distance from the centre of the turtlebot to the front of the cereal box when parked and scanning
#define ANGLE_INCREMENT 10                          // Degrees by which to increase in search for optimal goal destination from the centre
#define MAX_ANGLE 60                                // Max possible angle to deviate from centre optimal spot
// #define CAMERA_LIMIT 20                             // Seconds to be stuck trying to read an image
#define ARRIVAL_DELAY 1                             // Time to wait before starting to try to scan camera [s]
#define GOAL_TOLERANCE 0.03                         // Tolerance for goal destination when trying to make plan (x, and y)

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

    for (int i = 0; i < box_coordinates.size(); ++i){
        if (visited[i] == 1){ 				// Means location is unvisited
            std::vector<float> tmp = box_coordinates[i];
            possible_target.push_back(tmp);		// Add unviisted coordinates to possible_target vector
        }
    }
    
    
    // Find closest index 
    float closest_dist = 1000.0;
    int closest_idx_p = -1;  // Index of cloest coordinates in the possible_targets array
    int closest_idx = -1;  // Index of closest coordinates in the boxes_coordinates (AKA true) array
    
    for(int i = 0; i < sum; ++i) {
        float x = possible_target[i][0];
        float y = possible_target[i][1];

        float dx = x - robot_pose.x;
        float dy = y - robot_pose.y;
        float distance = std::sqrt(dx*dx+dy*dy);
        if (distance < closest_dist){
            ROS_INFO("Coordinate ID: %i", i);
            ROS_INFO("Distance: %f", distance);
            closest_dist = distance;
            closest_idx_p = i;
        }
    }
    for (int i = 0; i < box_coordinates.size(); ++i) {
        for (int j = 0; j < 3; j++) {
            std::cout << box_coordinates[i][j] << "\n" << possible_target[closest_idx_p][j];
        }
        ROS_INFO("we here");
    	if (box_coordinates[i] == possible_target[closest_idx_p]){
            ROS_INFO("Equal");
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


int checkRepeat(uint8_t results_array[5]){
    bool duplicate = false;
    for (int i = 0; i<5; i++){
        for (int n = i+1; n <5; n++){
            if (results_array[n] == results_array[i]){
                duplicate = true;
            }
        }
    }
    return duplicate;
}


// Duplicate =  True and therefore the last unvisited location is unique (not a repeat)
std::tuple<int, int> fillResults(uint8_t results_array[5]){
    uint8_t unvisited_idx = 55;
    uint8_t missing_id = 55;
    bool cases[4] = {0,0,0,0};

    for (int i = 0; i<5; i++){
        if (results_array[i] == 0){ // matches with raisin
            cases[0] = 1;
        }
        else if (results_array[i] == 1){ // matches with cinnamon
            cases[1] = 1;
        }
        else if (results_array[i] == 2){ // matches with rice
            cases[2] = 1;
        }
        else if (results_array[i] ==3){ // matches with blank
            cases[3] = 1;
        }
        else{ // no match then it's unvisited = 55
            unvisited_idx = i;
        }
    }
    for (int i = 0; i<4; i++){
        if (cases[i] ==0){
            missing_id = i;
        }
    }
    return std::tuple<int, int> {unvisited_idx, missing_id};
}




// !!!!!!!!!!!!!MAIN!!!!!!!!!!!!!!!!!!!!!!!!!!
int main(int argc, char** argv) {
    ros::init(argc, argv, "cereal_recognition");    // Initialize node name
    ros::NodeHandle n;
    ros::Rate loop_rate(LOOP_RATE);                 // Code will try to run at LOOP_RATE Hz

    RobotPose robot_pose(0,0,0);                     // Index as robot_pose.x, robot_pose.y, robot_pose.phi
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::poseCallback, &robot_pose);
    ros::ServiceClient move_client = n.serviceClient<nav_msgs::GetPlan>("/make_plan"); // Client for the make_plan service
    
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

    ImagePipeline image_pipeline(n);                 // Initialize image object and subscriber.

    std::chrono::time_point<std::chrono::system_clock> start; // Contest count down timer
    start = std::chrono::system_clock::now();

    // std::chrono::time_point<std::chrono::system_clock> camera_start; // Timer for camera to get result
    
    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::Quaternion q;
    nav_msgs::GetPlan srv;
    target_pose.header.frame_id = "map";
    srv.request.tolerance = GOAL_TOLERANCE;                   // Tolerance of destination goal in [m]

    // uint64_t camera_seconds_elapsed = 0;
    uint64_t seconds_elapsed = 0;
    uint8_t results[5] = {255, 255, 255, 255, 255};           // 0: Raisin Bran, 1: Cinnamon Toast Crunch, 2: Rice Krispies, 3: Blank
    int accum = 255;
    int target = 255;
    int label = 255;
    int offset_angle = 0;
    uint8_t yaw_adjust = 0;
    float trajectory_x = 0;
    float trajectory_y = 0;
    float trajectory_phi = 0;
    float cereal_x = 0;
    float cereal_y = 0;
    float cereal_yaw = 0;
    bool visited[5] = {1, 1, 1, 1, 1};
    bool is_positive = 0;
    bool is_back_at_start = false;
    bool arrived_at_target = 0;

    uint8_t missing_id = 255;
    uint8_t missing_label = 255;

    

    // Execute strategy.
    while(ros::ok() && seconds_elapsed <= 300 && (!is_back_at_start)) {
        ros::spinOnce();

        if (accum > 0) {
            target = getNearestNeighbour(robot_pose, boxes.coords, visited); //output uint8_t from 0-4
            ROS_INFO("target: %i ", target);
            if (target > 4) continue;

            cereal_x = boxes.coords[target][0];
            cereal_y = boxes.coords[target][1];
            cereal_yaw = boxes.coords[target][2];

            int i = 0;
            while (offset_angle <= MAX_ANGLE) {

                is_positive = i % 2;
                if (is_positive) {                                      // Odd number for i, positive angle offset
                    offset_angle = ANGLE_INCREMENT * (i + 1)/2;         // Increment of ANGLE_INCREMENT degrees
                    yaw_adjust = (offset_angle * M_PI/180) + cereal_yaw;
                } 
                
                else { //even numbers, include the initial 0 and the sunsequent clockwise locations in negative driection
                    offset_angle = ANGLE_INCREMENT * i/2;
                    yaw_adjust = -(offset_angle * M_PI/180) + cereal_yaw;
                }

                trajectory_x = VISION_RADIUS*cosf(yaw_adjust) + cereal_x;
                trajectory_y = VISION_RADIUS*sinf(yaw_adjust) + cereal_y;
                trajectory_phi = (yaw_adjust + M_PI); //!!!!!!!Check what the bounds of this need to be

                // try to make plan with trajectory_x, trajectory_y, trajectory_phi, if so, break loop
                target_pose.header.stamp = ros::Time::now();
                target_pose.pose.position.x = trajectory_x; // index coordinates file;
                target_pose.pose.position.y = trajectory_y; // index coordinates file;

                q = tf::createQuaternionMsgFromYaw(trajectory_phi);
                target_pose.pose.orientation.x = 0.0;
                target_pose.pose.orientation.y = 0.0;
                target_pose.pose.orientation.z = q.z;
                target_pose.pose.orientation.w = q.w;

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

                move_client.call(srv);
                if (srv.response.plan.poses.size() > 0) break;
                i++;
            }
        }
        else { // All labels at locations are known
            trajectory_x = 0;
            trajectory_y = 0;
            trajectory_phi = 0;
            is_back_at_start = true;
            ROS_INFO("Went into else");
        }
        ROS_INFO("TARGETS x: %f, y: %f, phi: %f, offset angle: %i", trajectory_x, trajectory_y, trajectory_phi, offset_angle);
        arrived_at_target = Navigation::moveToGoal(trajectory_x, trajectory_y, trajectory_phi);

        if((arrived_at_target) && (!is_back_at_start)) {
            ros::Duration(ARRIVAL_DELAY).sleep();               // Small pause to account for momentum shift
            // camera_start = std::chrono::system_clock::now();
            // camera_seconds_elapsed = 0;
            // label = 255;
            // while ((camera_seconds_elapsed < CAMERA_LIMIT) && (label == 255)) {
            //     label = image_pipeline.getTemplateID(boxes); //int from 0-3
            //     camera_seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-camera_start).count(); //time elapsed trying to read image
            // }

            label = image_pipeline.getTemplateID(boxes); //int from 0-3
            ROS_INFO("label: %i", label);
            visited[target] = 0;
            results[target] = label;// OpenCV, !!!Ask OpenCV people to return 255 if no good image found. Move onto next goal

            accum = std::accumulate(visited, visited + 5, accum=0);
            ROS_INFO("Locations visited: %i", accum);
            if (accum == 1) {
                if (checkRepeat(results)) { //input is visited array, output is boolean
                // if (true) { //input is visited array, output is boolean
                    ROS_INFO("there are repeats");
                    std::tie(missing_id, missing_label) = fillResults(results); // Input is results array, output is results array
                    results[missing_id] = missing_label;
                    accum = 0;
                }
            }
        }
        seconds_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); // Count how much time has passed
        loop_rate.sleep();                              // Delay function for 100ms

    }
    return 0;
}
