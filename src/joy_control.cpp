// Include ROS library
#include "ros/ros.h"
// Include msg
#include "sensor_msgs/Joy.h"
// Include IO library
#include "wiringPi.h"
#include "softPwm.h"
// Include motor class
#include "motor.h"

// M1 20	// wPi = p28 bcm = 20 	phPin = 38
// M2 21	// wPi = p29 bcm = 21 	phPin = 40
// M3 6     // wPi = p22 bcm = 6	phPin = 31
// M4 13	// wPi = p23 bcm = 13	phPin = 33
// EN1 26	// wPi = p25 bcm = 26	phPin = 37
// EN2 12	// wPi = p26 bcm = 12	phPin = 32

Motor m_left = Motor(20, 21, 26); // Deklarasi pin motor (pin maju, pin mundur, pin pwm)
Motor m_right = Motor(6, 13, 12);

double x_axis;
double y_axis;
int en1, en2;
double ex1, ex2, ey1, ey2;

void joy_callback(const sensor_msgs::Joy::ConstPtr &msg){
    x_axis = msg->axes[0];
    y_axis = msg->axes[1];
    
    if (y_axis>0){
        m_left.forward();
        m_right.forward();
        
        }
    else if(y_axis<0){
        m_left.backward();
        m_right.backward();
    }
    else{
        m_left.stop();
        m_right.stop();
    };
    double y_map = std::abs(y_axis)*100;
    double x_map = std::abs(x_axis)*100;
    
    if (x_axis>0){
        en1=y_map + x_map;
        en2=y_map - x_map;
    }
    else if(x_axis<0){
        en1=y_map - x_map;
        en2=y_map + x_map;
    }
    ROS_INFO("Duty Cycle");
    ROS_INFO("L: %d \tR: %d\n", en1, en2);
    /*
    // Old school way not done
    
    // ey1 = (y_axis/2)*100;
    // ey2 = (y_axis/2)*100;
    // ex1 = (1-x_axis);
    // ex2 = (std::abs(-1)-(-x_axis));

    // en1 = (ex1/2)*100;
    // en2 = (ex2/2)*100;
    
    // ROS_INFO("Axes values");
    // ROS_INFO("X: %f \tY: %f", x_axis, y_axis);
    // ROS_INFO("Ratio of both motors");
    // ROS_INFO("Left: %f \tRight: %f", ex1, ex2);
    // ROS_INFO("Additional y values");
    // ROS_INFO("Left: %f \tRight: %f", ey1, ey2);
    // ROS_INFO("Duty Cycle");
    // ROS_INFO("L: %d \tR: %d\n", en1, en2);*/
}

int main(int argc, char **argv)
{
    // setup();
    ros::init(argc, argv, "joy_control_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/joy", 1000, joy_callback);
    ROS_INFO("Node is Running");
    while(ros::ok()){
        ros::spin();
    }
    return 0;
}

// void setup(){
//     m_left.init();
//     m_right.init();
// }