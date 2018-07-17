/*
 * Team: Cyberdyne Industries
 * Mohammad Hassnain
 * Reyniel Maglian
 * Stephen Chan
 * Vincent Campbell
 *
 * Source code that we modified:
 *
 * Author: Anis Koubaa
 * Year: 2015
 * Course: Introduction to Mobile Robots (CS460)
 * University: Prince Sultan University (Saudi Arabia)
 *
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;


const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double setAbsoluteOrientation(double desired_angle);
//double clean ();
double getDistance(double x1, double y1, double x2, double y2);
//void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);


int main(int argc, char **argv)
{
	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;
	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	turtlesim::Pose goal_pose;

	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);


	cout << "------------------ ROS Turtle Rectangle Project ------------------- " << endl;
	move(7.0, 4.0, 1);
	rotate(degrees2radians(90), degrees2radians(90), 1);
	move(7.0, 2.0, 1);
	rotate(degrees2radians(90), degrees2radians(90), 1);
	move(7.0, 4.0, 1);
	rotate(degrees2radians(90), degrees2radians(90), 1);
	move(7.0, 2.0, 1);
	rotate(degrees2radians(90), degrees2radians(90), 1);
	move(7.0, 2.0, 1);


	ros::Rate loop_rate(0.5);

	while (ros::ok()) {

		/** test your code here **/

		/** run the clean application afer you implement it*/
		double t0 = ros::Time::now().toSec(); //get current time before cleaning
		//clean();
		double t1 = ros::Time::now().toSec(); //get current time after cleaning
		ROS_INFO("Cleaning execution time: %.2f", (t1 - t0));
		return 0;

	}

	ros::spin();

	return 0;
}

/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */
void move(double speed, double distance, bool isForward) {
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x = abs(speed);
	else
		vel_msg.linear.x = -abs(speed);
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	} while (current_distance < distance);
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);

}

/**
 *  makes the robot turn with a certain angular velocity, for
 *  a certain distance in either clockwise or counter-clockwise direction
 */
void rotate(double angular_speed, double relative_angle, bool clockwise) {

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	if (clockwise)
		vel_msg.angular.z = -abs(angular_speed);
	else
		vel_msg.angular.z = abs(angular_speed);

	double t0 = ros::Time::now().toSec();
	double current_angle = 0.0;
	ros::Rate loop_rate(1000);
	do {
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1 - t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_angle <<", "<<relative_angle<<endl;
	} while (current_angle < relative_angle);
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

/**
 *  converts angles from degree to radians
 */

double degrees2radians(double angle_in_degrees) {
	return angle_in_degrees *PI / 180.0;
}

/**
 *  callback function to update the pose of the robot
 */

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message) {
	turtlesim_pose.x = pose_message->x;
	turtlesim_pose.y = pose_message->y;
	turtlesim_pose.theta = pose_message->theta;
}

/**
 *  turns the robot to a desried absolute angle
 */

double setAbsoluteOrientation(double desired_angle_radians) {

	double relative_angle_radians = desired_angle_radians - turtlesim_pose.theta;
	bool clockwise = ((relative_angle_radians < 0) ? true : false);
	cout << desired_angle_radians << "," << turtlesim_pose.theta << "," << relative_angle_radians << "," << clockwise << endl;
	rotate(abs(relative_angle_radians), abs(relative_angle_radians), clockwise);
}

/*
 * get the euclidian distance between two points
 */
double getDistance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
}
