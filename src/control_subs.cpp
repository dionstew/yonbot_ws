// include ROS library
#include "ros/ros.h"
// include msgs
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h" 
//include IO library
#include "wiringPi.h"
#include "softPwm.h"
//include c++ library
#include "math.h"

#define M1 20	// wPi = p28 bcm = 20 	phPin = 38
#define M2 21	// wPi = p29 bcm = 21 	phPin = 40
#define M3 6	// wPi = p22 bcm = 6	phPin = 31
#define M4 13	// wPi = p23 bcm = 13	phPin = 33
#define EN1 26	// wPi = p25 bcm = 26	phPin = 37
#define EN2 12	// wPi = p26 bcm = 12	phPin = 32

float x, linear_speed;
float z, angular_speed;
bool is_forward, is_cw;
void setup();
void move(float lin_speed, bool lin_dir);
void rotate(float ang_speed, bool ang_dir);
void stop();

void callback(const geometry_msgs::Twist::ConstPtr& msg){
	x = msg->linear.x;
	z = msg->angular.z;
	ROS_INFO("x : %f, z : %f", x, z);
	double time0 = ros::Time::now().toSec();
	double time1;
	if (x>0){
		is_forward = true;
		}
	else {
		is_forward = false;
		x = std::abs(x);
		};

	if (z>0){
		is_cw = false;
		z = std::abs(z);
		}
	else {
		is_cw = true;
		};
	linear_speed = x;
	angular_speed = z;
	
	do{
	time1 = ros::Time::now().toSec();
	if (linear_speed != 0 && angular_speed == 0){
		move(linear_speed, is_forward);
		}
	else if (linear_speed == 0 && angular_speed != 0){
		rotate(angular_speed, is_cw);	
		}
	else {
		stop();
		}
	}while(time1-time0 < 1);
	stop();
	ROS_INFO("ROBOT STOPS");
}

int main(int argc, char **argv){
	setup();
	ros::init(argc, argv, "control_subs_cpp");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/turtle1/cmd_vel", 1000, callback);
	ROS_INFO("Node is running");		

	while (ros::ok()){	
		ros::spin();
	}
	
	stop();
	return 0;
}

void setup(){
	wiringPiSetupGpio();
	pinMode(M1, OUTPUT);
	pinMode(M2, OUTPUT);
	pinMode(M3, OUTPUT);
	pinMode(M4, OUTPUT);
	pinMode(EN1, OUTPUT);
	pinMode(EN2, OUTPUT);
	softPwmCreate(EN1, 1, 100);
	softPwmCreate(EN2, 1, 100);
}

void move(float lin_speed, bool lin_dir){
	int gain_sp = lin_speed * 30;
	if (lin_dir){
		digitalWrite(M1, HIGH);
		digitalWrite(M2, LOW);
		digitalWrite(M3, HIGH);
		digitalWrite(M4, LOW);
		ROS_INFO("ROBOT MOVES FORWARD");
		}
	else{
		digitalWrite(M1, LOW);
		digitalWrite(M2, HIGH);
		digitalWrite(M3, LOW);
		digitalWrite(M4, HIGH);
		ROS_INFO("ROBOT MOVES BACKWARD");
		}
	softPwmWrite (EN1, gain_sp); /* change the value of PWM */
	softPwmWrite (EN2, gain_sp);
	}

void rotate(float ang_speed, bool ang_dir){
	int gain_sp = ang_speed * 30;
	if (ang_dir){
		digitalWrite(M1, HIGH);
		digitalWrite(M2, LOW);
		digitalWrite(M3, LOW);
		digitalWrite(M4, HIGH);
		ROS_INFO("ROBOT ROTATES RIGHT");
		}
	else{
		digitalWrite(M1, LOW);
		digitalWrite(M2, HIGH);
		digitalWrite(M3, HIGH);
		digitalWrite(M4, LOW);
		ROS_INFO("ROBOT ROTATES LEFT");
		}
	}

void stop(){
	digitalWrite(M1, LOW);
	digitalWrite(M2, LOW);
	digitalWrite(M3, LOW);
	digitalWrite(M4, LOW);
	softPwmWrite (EN1, 0); /* change the value of PWM */
	softPwmWrite (EN2, 0);
	}
