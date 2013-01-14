/*
 * HumanFollower.h
 *
 *  Created on: Jan 12, 2013
 *      Author: t.posluszny
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Image.h"
#include "tf/tfMessage.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

#define MAX_LINEAR_VELOCITY 0.1 // m/s
#define MAX_ANGULAR_VELOCITY 0.3 // rad/s
#define MIN_DISTANCE 1.0 // meters
#define MAX_DETECTION_TIME_GAP 2.0 // seconds

#ifndef HUMANFOLLOWER_H_
#define HUMANFOLLOWER_H_

class HumanFollower {
public:
	// 0 if no human found, human position relative to robot, robot position from odometry topic
	double mHumanPositionX, mHumanPositionY;
	double mRobotPositionX, mRobotPositionY;
	// distance from nearest obstacle
	uint mDistance;

	ros::Time mLastDetectionTime;

	virtual ~HumanFollower();

	static HumanFollower* getInstance();

	// navigation algorithm
	void navigate();

private:
	HumanFollower();
	// getting robot odometry information
	static void odometryCallback(const nav_msgs::Odometry& msg);
	// getting human position
	static void humanpositionCallback(const tf::tfMessage& msg);
	// getting human position
//	static void depthCallback(const sensor_msgs::Image& msg);

	// robot velocity publisher
	ros::Publisher mPubRobotVelocity;
	// human position subscriber
	ros::Subscriber mSubHumanPosition;
	// robot position subscriber (odometry)
	//ros::Subscriber mSubRobotPosition;
	// depth image from kinect subscriber
//	ros::Subscriber mSubDepth;

	static HumanFollower *instance;
};

#endif /* HUMANFOLLOWER_H_ */
