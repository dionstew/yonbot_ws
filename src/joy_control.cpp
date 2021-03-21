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
bool m_l_dir, m_r_dir;

void joy_callback(const sensor_msgs::Joy::ConstPtr &msg){
    x_axis = msg.axes[0];
    y_axis = msg.axes[1];
    
    en1 = std::abs(-1-(-x_axis))*100;
    en2 = (1-x_axis)*100;
    
    if (y_axis > 0){
        m_l_dir = true;
        m_r_dir = true;    
    }

    else if (y_axis<0){
        m_l_dir = false;
        m_r_dir = false;
    };

    // Send result to move the motor
    m_left.move(m_l_dir, en1);
    m_right.move(m_r_dir, en2);
}

int main(int argc, char const *argv[])
{
    setup();
    ros::init(argc, argv, "joy_control_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subsrcibe("/joy", 1000, joy_callback);
    ROS_INFO("Node is Running");
    while(ros::ok()){
        ros.spin();
    }
    return 0;
}

void setup(){
    m_left.init();
    m_right.init();
}