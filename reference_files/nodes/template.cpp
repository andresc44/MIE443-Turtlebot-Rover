#include <ros/ros.h> //always keep
#include <std_msgs/Int64.h> //topic data types
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/SetBool.h>

class ClassName {
    private:
        int foo; //initialize variables
        int bar;
        ros::Publisher pub1; //create publisher object
        ros::Publisher pub2; //create publisher object
        ros::Subscriber sub1; //subscriber object
        ros::Subscriber sub2; //subscriber object
        ros::ServiceServer serv1; //service object
    
    public:
    ClassName(ros::NodeHandle *nh) {
        foo = 0;
        pub1 = nh->advertise<std_msgs::Int64>("/pubTopic1", 10); //publish to topic1, and set datatype
        pub2 = nh->advertise<std_msgs::Int64>("/pubTopic2", 10); //publish to topic2, and set datatype
        sub1 = nh->subscribe("/subTopic1", 10, //topic, queueSize, 
            &ClassName::callback_function_sub1, this); //callback function
        sub2 = nh->subscribe("/subTopic2", 15, //topic, queueSize, 
            &ClassName::callback_function_sub2, this); //callback function
        serv1 = nh->advertiseService("/serv1", 
            &ClassName::callback_function_serv, this);
    }

    void callback_function_sub1(const std_msgs::Int64& msg) {
        //code to process or clean data before passing to foo
        foo = msg.data;
        std_msgs::Int64 new_msg;
        new_msg.data = foo;
        pub1.publish(new_msg); //publish entire msg, not msg.data Can publish if pub is dependent on 1 sub
    }

    void callback_function_sub2(const std_msgs::Int64& msg) {
        //code to process or clean data before passing to foo
        bar = msg.data;
        std_msgs::Int64 new_msg;
        new_msg.data = bar;
        pub1.publish(new_msg); //publish entire msg, not msg.data
    }
    void publishAll() {

        std_msgs::Bool new_msg1;
        std_msgs::UInt8 new_msg2;
        if (foo + bar > 80) {
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

    bool callback_function_serv(std_srvs::SetBool::Request &req, 
                                std_srvs::SetBool::Response &res)
    //code for the server. It receives the requests, and outputs the response
    //could use for infrequent topic changes
    
    {
        if (req.data) {
            foo = 0;
            res.success = true;
            res.message = "Counter has been successfully reset";
        }
        else {
            res.success = false;
            res.message = "Counter has not been reset";
        }
        return true;
    }
};

int main (int argc, char **argv)
{
    int sleep_rate = 10; //10 Hz
    ros::init(argc, argv, "nodeName"); //Name of the node
    ros::NodeHandle nh;
    // ros::param::get("/my_param_name", sleep_rate);

    ros::Rate r(sleep_rate); // Hz
    ClassName cn = ClassName(&nh);
    while (ros::ok())
        {
            //ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1); To set different rates for subs
            //ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);
            //If rates differ throughout, then can associate publish events into timed callbacks
            
            ros::spinOnce();
            cn.publishAll();
            r.sleep();
        }
    return 0;
}