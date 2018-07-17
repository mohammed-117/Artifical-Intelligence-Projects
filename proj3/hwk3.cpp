/*
 * Team: Cyberdyne Industries
 * Mohammad Hassnain
 * Reyniel Maglian
 * Stephen Chan
 * Vincent Campbell
 *
 * We are modifying and our robot_cleaner.cpp file from last
   assignment to complete assignment #3
 * a) We spawn the turtles
 * b) Then we move the main turtle to go and capture the target turtles
 * c) Hueristic Search/Function comes into play and moves the main
	  turtle to collect the turtles.
 */

#include "ros/ros.h"
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <iostream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;


const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

int max_villain_turtles = 4;
int max_target_turtles = 3;

const double PI = 3.14159265359;

void move(double speed, double distance, bool isForward);
void rotate(double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
double setAbsoluteOrientation(double desired_angle);
//double clean ();
double getDistance(double x1, double y1, double x2, double y2);
//void moveGoal(turtlesim::Pose goal_pose, double distance_tolerance);

struct TurtleClass
{
	int x_point;
	int y_point;
	string Sname;
	bool a_villain; //Keeps track of its a villain or not

};

//Start of Main--------------------------------
int main(int argc, char **argv)
{
	cout << "------------------ ROS Turtle Rectangle Project ------------------- " << endl;

	// Initiate new ROS node named "talker"
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;

	double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;
	turtlesim::Pose goal_pose;
	/****************************************/

	// turtle struct object
	TurtleClass tu[7];

	// turtle target and villian names
	const char SpawnName[][5] = { "T1","T2","T3",
								"X1","X2","X3","X4" };
	int x, y;
	int theta = 0;
	char sCommand[50];
	char command[50];
	char str[10];
	char str2[10];

	// initialize random seed
	srand(time(NULL));

	/***********************************************************/

		// Turn Pen to draw black initial line
	strcpy(sCommand, "rosservice call /turtle1/set_pen 0 0 0 1 0");
	system(sCommand);

	// Move to start location for Turtle1
	strcpy(sCommand, "rosservice call /turtle1/teleport_absolute 0 11 0.0");
	system(sCommand);


	// Clear lines on map
	strcpy(sCommand, "rosservice call clear");
	system(sCommand);

	// Turn Pen to draw original line color (white)
	strcpy(sCommand, "rosservice call /turtle1/set_pen 0 0 0 1 1");
	system(sCommand);






	/*********************************************************/
	for (int i = 0; i < 7; i++)
	{
		// Random location generator
		x = 2 + static_cast <int> (rand()) / (static_cast <int> (RAND_MAX / (10 - 2)));
		y = 2 + static_cast <int> (rand()) / (static_cast <int> (RAND_MAX / (10 - 2)));

		// Check turtle spawn in same location
		// also keep distance from other spawned turtles
		if (((tu[i].x_point != tu[i - 1].x_point) && (tu[i].y_point != tu[i - 1].y_point))
			&& (((tu[i].x_point + 1) != tu[i - 1].x_point) && ((tu[i].y_point + 1) != tu[i - 1].y_point))
			&& (((tu[i].x_point - 1) != tu[i - 1].x_point) && ((tu[i].y_point - 1) != tu[i - 1].y_point))
			)
		{


			// saving values in struct
			tu[i].x_point = x;
			tu[i].y_point = y;

			sprintf(str, "%d", x);
			sprintf(str2, "%d", y);

			strcpy(command, "rosservice call /spawn ");
			strcat(command, str);  // x coordinate
			strcat(command, " ");
			strcat(command, str2); // y coordinate
			strcat(command, " ");
			strcat(command, "0 "); // theta rotation
			strcat(command, SpawnName[i]);

			tu[i].Sname = SpawnName[i];
			if (i > 2) {
				tu[i].a_villain = true;
			}
			else
			{
				tu[i].a_villain = false;
			}

			// Spawn Turtle Call
			system(command);

			cout << "Spawn Turtle " << (i + 1) << " coordinates(x,y,z): " << "(" << fixed << setprecision(2) << x << "," << y << "," << theta << ")" << endl;

		}

	}


	cout << "-------------------STRUCT SAVED : Turtle Names---------------------" << endl;

	for (int h = 0; h < 7; h++)
	{
		cout << "Turtle Name: " << tu[h].Sname << endl;
	}

	/****************************************/


	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);


	ros::Rate loop_rate(0.5);

	while (ros::ok()) {
		/** run the clean application afer you implement it*/
		double t0 = ros::Time::now().toSec(); //get current time before cleaning
		//clean();
		double t1 = ros::Time::now().toSec(); //get current time after cleaning
		ROS_INFO("Cleaning execution time: %.2f", (t1 - t0));
		return 0;
	}


	//------------------Heuristic Search---------
	/*
	1. Find the target with smallest distance from starting point.
	2. Move to that target and check for distance of nearest villain turtle.
	3. If turtle is within distance rotate 45 degrees and increase distance by 1.
	4. when distance is about .5 then target is captured.
	5. Begin rotation.
	6. Set out course for other turtles.
	*/
	int i = 0;
	int array_of_villain_distances[4]; //This array has the distance of each villain from the turtle initial point
	int array_of_target_distances[3]; // This array has the distance of each target from turtle initial point.

	//Find the distance from starting point to each target and place it in array
	double smallest_target_distance = 0;
	double total_distance_traveled = 0.0;
	for (int i = 0; i < 3; i++)
	{
		array_of_target_distances[i] = getDistance(turtlesim_pose.x, turtlesim_pose.y, tu[i].x_point, tu[i].y_point);

		if (i == 0) {
			smallest_distance = array_of_target_distances[i];
		}
		else {
			if (smallest_distance > array_of_target_distances[i]) {
				smallest_distance = array_of_target_distances[i];
			}
		}
	}


	while (max_target_turtles != 0) //Loop until there are no villain turtles left
	{
		//Perform move:
		if (turtlesim_pose.x < 11.0 && turtlesim_pose.y < 11.0) //Boundary check
		{
			turtlesim::Pose goal_pose;
			goal_pose.x = array_of_villain_distances[i].x;
			goal_pose.y = array_of_villain_distances[i].y;

			total_distance_traveled = moveGoal(goal_pose, .5); // This will move the turtle and kill it.
			ROS_INFO_STREAM("Turtle Captured");

			checkIfClose(turtlesim_pose, tu);
			rotate(degrees2radians(45), degrees2radians(45), 1);
		}
		max_villain_turtles--;
		i++;
	}
	//--------------End of Heuristic-----------------


	ros::spin();
	return 0;

}
//End of Main--------------------


//Functions
/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */

 //Check if turtle is close to villains.
void checkIfClose(TurtleSim::Pose turtlesim_pose, TurtleClass turtles[]) {
	for (int i = 3; i < 7li++)
	{
		//Move the turtle before it gets close to .5 distance of a villain turtle.
		if (getDistance(turtlesim_pose.x, turtlesim_pose.y, villain_turtles[i].x_point, villain_turtles[i].y_point) <= .10) {
			rotate(degrees2radians(90), degrees2radians(90), 1);
			move(2.0, 2.0, 1);
		}
	}
}

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

//Constantly check if the turtle is close to a villain or not
bool checkDistance(TurtleClass original_turtle, TurtleClass villain_turtle) {
	if (getDistance(original_turtle.x, original_turtle.y, villain_turtle.x, villain_turtle.y) <= .5) {
		return false; // Villain can capture.
	}
	else {
		return true; //Turtle is still safe.
	}
}


int moveGoal(turtlesim::Pose  goal_pose, double distance_tolerance) {
	//We implement a Proportional Controller. We need to go from (x,y) to (x',y'). Then, linear velocity v' = K ((x'-x)^2 + (y'-y)^2)^0.5 where K is the constant and ((x'-x)^2 + (y'-y)^2)^0.5 is the Euclidian distance. The steering angle theta = tan^-1(y'-y)/(x'-x) is the angle between these 2 points.
	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(10);
	do {
		//linear velocity
		vel_msg.linear.x = 1.5*getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y);
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		//angular velocity
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 4 * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);



		velocity_publisher.publish(vel_msg);


		ros::spinOnce();
		loop_rate.sleep();

	} while (getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);
	cout << "end move goal" << endl;
	vel_msg.linear.x = 0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);

	return getDistance(turtlesim_pose.x, turtlesim_pose.y, goal_pose.x, goal_pose.y)

}



double clean() {
	//return the total distance traveled by the robot
	//calculate the distance as the of segment lengths or sum of distance between visited points.
}
