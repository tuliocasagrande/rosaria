#include <ros/ros.h>
#include <signal.h>
#include <tf/tf.h>
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"

#define CDEFAULT "\x1B[0m"
#define CRED "\x1B[31m"
#define CYELLOW "\x1B[33m"

#define LEFT_SIDE -1
#define FORWARD 0
#define RIGHT_SIDE 1

using namespace std;

using geometry_msgs::Point32;
using geometry_msgs::Twist;
using sensor_msgs::Imu;
using sensor_msgs::PointCloud;
using nav_msgs::Odometry;

/* Robot control */
bool stateChanged(Point32 currentImu, Point32 oldImu);
void reportAction();

/* Android Teleoperation Functions */
void imuCallback(const Imu::ConstPtr& ansmsg);
float processImu(float measuredImu);

/* Speed functions */
void poseCallback(const Odometry::ConstPtr& ansmsg);
bool isMoving();

/* Sonar Functions */
void sonarCallback(const PointCloud::ConstPtr& ansmsg);
double pointDistance(Point32 p);
double wallFollow(int side);
bool isCorner(int side);
bool nearWall(int side);
bool imminentFrontalCollision();

/* Global variables */
Point32 measuredImu;
Point32 measuredSonar[16];
double processedSonar[16];
Point32 measuredSpeed;
double measuredYawAngle;

void stopRobot(int sig) {
	// Publish a stop message to the robot
	ros::NodeHandle n;
	ros::Publisher velPub = n.advertise<Twist>("/RosAria/cmd_vel", 1);
	Twist velMsg;
	velMsg.linear.x = 0;
	velMsg.angular.z = 0;
	velPub.publish(velMsg);

	ros::shutdown();
}

int main(int argc, char** argv) {

	ros::init(argc, argv, "AndroidTeleop");
	ros::NodeHandle n;
	signal(SIGINT, stopRobot);
	cout << "Online!\n";

	Point32 currentImu, oldImu;
	Twist velMsg;

	ros::Publisher velPub = n.advertise<Twist>("/RosAria/cmd_vel", 1);

	ros::Subscriber imuSub = n.subscribe<Imu>("/android/imu", 1, imuCallback);
	ros::Subscriber sonarSub = n.subscribe<PointCloud>("/RosAria/sonar", 1, sonarCallback);
	ros::Subscriber poseSub = n.subscribe<Odometry>("/RosAria/pose", 1, poseCallback);

	/* The while will run at 10hz */
	ros::Rate loopRate(10);

	/* Get sensors callbacks before doing anything */
	ros::spinOnce();

	while (ros::ok()) {
		currentImu.x = processImu(measuredImu.x);
		currentImu.y = processImu(measuredImu.y);

		// User control
		if (currentImu.x > 0.2 || currentImu.x < -0.2 ||
		    currentImu.y > 0.2 || currentImu.y < -0.2) {
			if (stateChanged(currentImu, oldImu)) {
				velMsg.linear.x = currentImu.y;
				velMsg.angular.z = currentImu.x;
				velPub.publish(velMsg);
			}

		// Autonomous control
		} else {

			if (imminentFrontalCollision()) {
				velMsg.linear.x = 0;
				velPub.publish(velMsg);

			} else if (nearWall(FORWARD)) {
				velMsg.linear.x = 0.5;
				velPub.publish(velMsg);

			} else {
				velMsg.linear.x = 1.0;
				velPub.publish(velMsg);
			}

			// The robot needs to turn 90 degrees
			if (isCorner(RIGHT_SIDE)) {
				velMsg.linear.x = 0.5;
				velMsg.angular.z = -0.7853; // quarter pi
				velPub.publish(velMsg);
				ros::Duration(0.5).sleep(); // this value may change depending on hardware

			} else if (isCorner(LEFT_SIDE)) {
				velMsg.linear.x = 0.5;
				velMsg.angular.z = 0.7853; // quarter pi
				velPub.publish(velMsg);
				ros::Duration(0.5).sleep(); // this value may change depending on hardware
			}

			// Ensure parallelism with wall
			if (nearWall(RIGHT_SIDE)) {
				if (imminentFrontalCollision()) velMsg.angular.z = 0.5;
				else if (nearWall(FORWARD) && !nearWall(LEFT_SIDE)) velMsg.angular.z = 0.5;
				else velMsg.angular.z = wallFollow(RIGHT_SIDE);

			} else if (nearWall(LEFT_SIDE)) {
				if (imminentFrontalCollision()) velMsg.angular.z = -0.5;
				else if (nearWall(FORWARD) && !nearWall(RIGHT_SIDE)) velMsg.angular.z = -0.5;
				else velMsg.angular.z = wallFollow(LEFT_SIDE);

			// Far from any wall
			} else {
				if (nearWall(FORWARD)) velMsg.angular.z = 0.5;
				else velMsg.angular.z = 0;
			}

			velPub.publish(velMsg);
		}

		reportAction();
		oldImu = currentImu;

		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

/* ============================== Robot control ============================= */
bool stateChanged(Point32 currentImu, Point32 oldImu) {
	if (currentImu.x != oldImu.x) return true;
	if (currentImu.y != oldImu.y) return true;
	return false;
}

void reportAction() {

	cout << "========================================\n";
	cout << "Linear vel: " << measuredSpeed.x << ", angular vel: " << measuredSpeed.z << "\n";

	// Init map
	int map_size = 40;
	bool map[map_size][map_size];
	for (int i = 0; i < map_size; i++)
		for (int j = 0; j < map_size; j++)
			map[i][j] = false;

	// Adding sonar info
	for (int i = 0; i < 16; i++) {
		int mappedX = (int) (measuredSonar[i].x * -4 + 20.0);
		int mappedY = (int) (measuredSonar[i].y * -4 + 20.0);

		if (mappedX > map_size - 1) mappedX = map_size - 1;
		if (mappedX < 0) mappedX = 0;

		if (mappedY > map_size - 1) mappedY = map_size - 1;
		if (mappedY > map_size - 1) mappedY = 0;

		map[mappedX][mappedY] = true;
	}

	// Printing map
	for (int i = 0; i < map_size; i++) {
		for (int j = 0; j < map_size; j++) {

			if (map[i][j]) {
				if (i > 17 && i < 22 &&	j > 17 && j < 22)
					printf(CRED "\u2588" CDEFAULT);

				else if (i > 15 && i < 24 &&	j > 15 && j < 24)
					printf(CYELLOW "\u2588" CDEFAULT);

				else if (map[i][j]) printf("\u2588");

			// Printing robot
			} else if ((i == 19 || i == 20) && (j == 19 || j == 20)) {
				printf("\u2591");
				continue;

			} else printf(" ");
		}
		printf("\n");
	}
}

/* ==================== Android Teleoperation Functions ===================== */
void imuCallback(const Imu::ConstPtr& ansmsg) {
	measuredImu.x = ansmsg->linear_acceleration.x;
	measuredImu.y = ansmsg->linear_acceleration.y;
	measuredImu.z = ansmsg->linear_acceleration.z;
}

float processImu(float measuredImu) {
	int normalized = (int) measuredImu;

	int coef = 1;
	if (normalized < 0) {
		coef = -1;
		normalized *= coef;
	}

	/* Deadzone */
	if (normalized < 2) return 0.0;

	/* Gradative speed */
	if (normalized == 2) return 0.1 * coef;
	if (normalized == 3) return 0.2 * coef;
	if (normalized == 4) return 0.3 * coef;
	if (normalized == 5) return 0.4 * coef;
	if (normalized == 6) return 0.6 * coef;
	if (normalized == 7) return 0.8 * coef;

	/* Max speed */
	if (normalized > 7) return 1.0 * coef;
}

/* ============================ Speed Functions ============================ */
void poseCallback(const Odometry::ConstPtr& ansmsg){
	measuredSpeed.x = ansmsg->twist.twist.linear.x;
	measuredSpeed.z = ansmsg->twist.twist.angular.z;

	tf::Pose pose;
	tf::poseMsgToTF(ansmsg->pose.pose, pose);
	measuredYawAngle = tf::getYaw(pose.getRotation());
}
bool isMoving() {
	if (measuredSpeed.x > 0) return true;
	return false;
}

/* ============================ Sonar Functions ============================= */
void sonarCallback(const PointCloud::ConstPtr& ansmsg) {
	for (int i = 0; i < 16; i++) {
		measuredSonar[i] = ansmsg->points[i];
		processedSonar[i] = pointDistance(measuredSonar[i]);
	}

}
double pointDistance(Point32 p){
	return sqrt( pow(p.x,2) + pow(p.y,2) );
}
double wallFollow(int side) {

	double wallDifference, distanceFromWall;
	if (side == RIGHT_SIDE) {
		wallDifference = measuredSonar[7].y - measuredSonar[8].y;
		distanceFromWall = measuredSonar[7].y * -1.0;
	} else {
		wallDifference = measuredSonar[0].y - measuredSonar[15].y;
		distanceFromWall = measuredSonar[0].y;
	}
	// counterclockwise movements
	if (wallDifference > 1.0) return 1.0;
	if (wallDifference > 0.5) return 0.5;
	if (wallDifference > 0.05) return 0.25;
	if (wallDifference > 0.005) return 0.05;

	// clockwise movements
	if (wallDifference < -1.0) return -1.0;
	if (wallDifference < -0.5) return -0.5;
	if (wallDifference < -0.05) return -0.25;
	if (wallDifference < -0.005) return -0.05;

	// If possible, try to maintain a safe distance from walls
	if (!nearWall(side * -1)) { // RIGHT_SIDE=1, LEFT_SIDE=-1
		if (distanceFromWall > 2.0) return -0.75 * side;
		if (distanceFromWall > 1.5) return -0.25 * side;
		if (distanceFromWall > 1.2) return -0.1 * side;
		if (distanceFromWall < 0.8) return 0.1 * side;

	// Between two walls (corridor or labyrinth)
	} else {
		double bothWallsDifference = measuredSonar[0].y - measuredSonar[7].y * -1.0;
		if (bothWallsDifference > 0.05) return 0.1;
		if (bothWallsDifference > 0.005) return 0.05;
		if (bothWallsDifference < -0.05) return -0.1;
		if (bothWallsDifference < -0.005) return -0.05;
	}

	// Near and parallel to the wall
	return 0.0;
}

bool isCorner(int side) {
	if (side == RIGHT_SIDE) {
		if (processedSonar[6] > (-2 * measuredSonar[7].y) &&
		    measuredSonar[7].y > -1.2 && measuredSonar[8].y > -1.2) return true;
	} else if (side == LEFT_SIDE) {
		if (processedSonar[1] > (2 * measuredSonar[0].y) &&
		    measuredSonar[0].y < 1.2 && measuredSonar[15].y < 1.2) return true;
	}
	return false;

}
bool nearWall(int side) {
	if (side == RIGHT_SIDE) {
		if (measuredSonar[7].y > -3.0 || measuredSonar[8].y > -3.0) return true;
	} else if (side == LEFT_SIDE) {
		if (measuredSonar[0].y < 3.0 || measuredSonar[15].y < 3.0) return true;
	} else if (side == FORWARD) {
		if (processedSonar[3] < 2.5 || processedSonar[4] < 2.5) return true;
		if (processedSonar[2] < 1.5 || processedSonar[5] < 1.5) return true;
	}
	return false;
}
bool imminentFrontalCollision() {
	if (processedSonar[3] < 0.8 || processedSonar[4] < 0.8) return true;
	if (processedSonar[2] < 0.4 || processedSonar[5] < 0.4) return true;
	return false;
}

