#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>



#define N_BUMPER (3) //Number of bumpers on kobuki base
#define RAD2DEG(rad) ((rad) * 180. / M_PI) //coversion function 
#define DEG2RAD(deg) ((deg) * M_PI / 180.) //inverse conversion function

uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT]; // kobuki_msgs::BumperEvent::PRESSED if bumper is pressed, kobuki_msgs::BumperEvent::RELEASED otherwise

float angular = 0.0; //declare rotational vel (z) rad/s
float linear = 0.0; //declare linear velocity (x) m/s
float posX = 0.0, posY = 0.0, yaw = 0.0; //initialize odom position
float minLaserDist = std::numeric_limits<float>::infinity(); //set minimum distance, volatile variable
int32_t nLasers=0, desiredNLasers=0, desiredAngle=5; //laser parameters

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
    // Access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
    bumper[msg->bumper] = msg->state; //assigns bumper array to match message based on msg message
	//fill with your code
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    minLaserDist = std::numeric_limits<float>::infinity(); //no minimum distance 
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment; //count how many lasers there are  per cycle
    desiredNLasers = desiredAngle*M_PI / (180*msg->angle_increment); //offset desired
    ROS_INFO("Size of laser scan array: %i and size of offset: %i", nLasers, desiredNLasers); //print statement
    
    if (desiredAngle * M_PI / 180 < msg->angle_max && -desiredAngle * M_PI / 180 > msg->angle_min) {
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx){
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }
    else { //edge cases?
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx) {
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
        }
    }

}
void odomCallback (const nav_msgs::Odometry::ConstPtr& msg)
{
    posX = msg->pose.pose.position.x; //get x position
    posY = msg->pose.pose.position.y; //get y position
    yaw = tf::getYaw(msg->pose.pose.orientation); //covert from quaternion to yaw
    tf::getYaw(msg->pose.pose.orientation); //not sure
    ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, RAD2DEG(yaw)); //print statement
    
//fill with your code
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener"); //node name
    ros::NodeHandle nh;                      //initialize node handler

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback); //subscribe to bump topic with callback
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback); //subscribe  to /scan topic
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback); //NEW
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1); //publish cmd_velocity as Twist message

    ros::Rate loop_rate(10); //code will try to run at 10 Hz

    geometry_msgs::Twist vel; //create message for velocities as Twist type

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start; //initialize timer
    start = std::chrono::system_clock::now(); //start the timer
    uint64_t secondsElapsed = 0; //variable for time that has passed
    

    while(ros::ok() && secondsElapsed <= 480) { //while ros master running and < 8 minutes
        ROS_INFO("Postion: (%f, %f) Orientation: %f degrees Range: %f", posX, posY, RAD2DEG(yaw), minLaserDist); //print current state of everything, !!!!!!!CHANGED 2 LINES
        ros::spinOnce(); //listen to all subscriptions once
        //fill with your code

        //
        // Check if any of the bumpers were pressed.
        bool any_bumper_pressed = false; //reset variable to false
        for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx) { //check all 3 bumpers
        any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED); //updates variable if ANY are pressed
        }
        //
        // Control logic after bumpers are being pressed.
        if (posX < 0.5 && yaw < M_PI / 12 && !any_bumper_pressed) { //move forward 20cm
            angular = 0.0; 
            linear = 0.2;
        }
        else if (yaw < M_PI / 2 && posX > 0.5 && !any_bumper_pressed) { //turn left
            angular = M_PI / 6;
            linear = 0.0;
        }
        else if (minLaserDist > 1. && !any_bumper_pressed) { //go straight and then decide if turning
            linear = 0.1;
            if (yaw < 17 / 36 * M_PI || posX > 0.6) { //turn left a lil
                angular = M_PI / 12.;
            }
            else if (yaw < 19 / 36 * M_PI || posX < 0.4) { //turn right a lil
                angular = -M_PI / 12.;
            }
            else { //don't turn
                angular = 0;
            }
        }
        else { //stop moving
            angular = 0.0;
            linear = 0.0;
        }

        vel.angular.z = angular; //set rotation to Twist
        vel.linear.x = linear; //set linear to Twist
        vel_pub.publish(vel); //publish cmd_vel to teleop mux input

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count(); //count how much time has passed
        loop_rate.sleep(); //delay function for 100ms
    }

    return 0;
}
