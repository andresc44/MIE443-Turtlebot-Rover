//syntax is package-name/message-type-name

#include <ros/ros.h> //always keep
#include <std_msgs/Int64.h> //topic data types
#include <std_msgs/Bool.h> // for top_half and axis of decision making
#include <std_msgs/UInt8.h> // operation mode
#include <geometry_msgs/PoseWithCovarianceStamped.h> // amcl_pose package
#include <move_base_msgs/MoveBaseAction.h> // for move_base
// move base action accepts goals from clients and attempts to move robot to specified position in the world
#include <actionlib/client/simple_action_client.h> // for move_base

//not really sure where this line belongs tbh, but its form the simple_nav_goals.cpp
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class ClassName {

    //defining private attributes 
    private:
        //defining variables

        int mode; //initialize mode of robot variables. It is an integer
        int current_pose;

        // creating publisher and subscriber objects
        ros::Publisher pub1; //create publisher object
        ros::Publisher pub2; //create publisher object
        ros::Publisher pub3; //create publisher object
        

        ros::Subscriber sub1; //subscriber object
        ros::Subscriber sub2; //subscriber object
        ros::ServiceServer serv1; //service object
    
    // defining publica attributes
    public:
    ClassName(ros::NodeHandle *nh) {
        mode = 0; // by default mode is set to 0, which will call the robot to localize

        //publish a total of 3 topics
        pub1 = nh->advertise<std_msgs::UInt8>("/operation_mode", 10); //publish to topic2, and set datatype
        pub2 = nh->advertise<std_msgs::Bool>("/axis of decision making", 10); //publish to topic2, and set datatype
        pub3 = nh->advertise<std_msgs::Bool>("/top_half", 10); // publishes whether the robot is in top half of map, returns true or false
        
        //subscribe to a total of 2 topics
        sub1 = nh->subscribe("/amcl_pose", 10, //topic, queueSize, 
            &ClassName::callback_subscribe_amcl, this); //callback function
            
        sub2 = nh->subscribe("/operation_mode", 10, //topic, queueSize, 
            &ClassName::callback_subscribe_mode, this); //callback function

        
    }

    void callback_subscribe_amcl(const std_msgs::Int64& msg) {
        //code to process or clean data before passing to mode
        mode = msg.data;
        std_msgs::Int64 new_msg;
        new_msg.data = mode;
    }

    void callback_subscribe_mode(const std_msgs::Int64& msg) {
        //code to process or clean data before passing to mode
        current_pose = msg.data;
        std_msgs::Int64 new_msg;
        new_msg.data = current_pose;
        //pub1.publish(new_msg); //publish entire msg, not msg.data
    }

    // one method of publishing data
    void publishAll() {

        std_msgs::Bool new_msg1;
        std_msgs::UInt8 new_msg2;
        if (mode + current_pose > 80) {
            new_msg1.data = true;
            new_msg2.data = 255;
        }
        else {
            new_msg1.data = false;
            new_msg2.data = 122;
        }
        pub1.publish(new_msg1);
        pub2.publish(new_msg2);
    }


    // define a call back function that sends goals depending on the mode
    //void since it doesn't return a value
    void sendGoal(){

        //mode 2 is moving to Loading Zone once localized
        if (mode=2){
            
            if 
            
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
                ROS_INFO("Hooray, the base succeeded in moving");

            //if move_base failed then output "sad ;("
            else
                ROS_INFO("The base failed to move ");


        }
        //mode 4 is moving to Drop Off Area once block has been secured
        else if(mode=4){

            //Manually comment in whichever drop off point you want

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

            ROS_INFO("Sending goal");
            ac.sendGoal(goal); // THIS ACTUALLY PUSHES THE GOAL MESSAGE TO THE MOVE_BASE NODE FOR PROCESSING

            ac.waitForResult(); // this command blocks everything else until the move_base action is done processing the goal

            //once goal is done processing we will check with if statement if the goal succeeded or failed and output a message acoordingly
            
            //if move_base succeeded then output "hooray"
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                ROS_INFO("Hooray, the base succeeded in moving");

            //if move_base failed then output "sad ;("
            else
                ROS_INFO("The base failed to move ");


        }

    }
};




int main(int argc, char** argv){
    
    int sleep_rate; //initialize sleep rate variable

    ros::init(argc, argv,"sendGoal"); //initalizes the ROS client libarary, only needs to be called once. node name is sendGoal
    ros::NodeHandle nh; // create node handle object to register program as a ros node to the ros master

    //tell the action client that we want to spin a thread by default
    //creates an action client called move_base. We'll use this client o
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up and ready to begin processing goals
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //create a goal to send to move_base (using move_base_msgs data type)
    move_base_msgs::MoveBaseGoal goal; //i'm going to guess that goal is just an xy position array

    
    goal.target_pose.header.frame_id = "map"; // SPECIFY COORDINATE FRAME TO MOVE RELATIVE TO
    goal.target_pose.header.stamp = ros::Time::now();

    //I don't know that we need to define a node handle, it snot 
    
    // ros::param::get("/my_param_name", sleep_rate);
    // ros::Rate r(sleep_rate); // Hz, this is the number of messages per second
    // ClassName cn = ClassName(&nh);


    while (ros::ok()) //keep repeating loop until user presses Ctrl+C
        {
            //ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1); To set different rates for subs
            //ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);
            //If rates differ throughout, then can associate publish events into timed callbacks
            
            //insert callback Function to update subscriber nodes
            

            ros::spinOnce(); //call the callback functions
            sendGoal()

            cn.publishAll(); //publish topics
            r.sleep(); // put a delay before repeating while loop
        }
    return 0;
}