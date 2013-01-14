/*
 * HumanFollower.cpp
 *
 *  Created on: Jan 12, 2013
 *      Author: t.posluszny
 */

#include <math.h>
#include "HumanFollower.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>

HumanFollower* HumanFollower::instance = NULL;

HumanFollower::HumanFollower() : mHumanPositionX(0), mHumanPositionY(0), mDistance(0) {
	ros::NodeHandle n;
	mSubHumanPosition = n.subscribe("/tf", 10, humanpositionCallback);
	//mSubRobotPosition = n.subscribe("RosAria/pose", 100, odometryCallback);
//	mSubDepth = n.subscribe("depth/image_raw", 10, depthCallback);
	mPubRobotVelocity = n.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 10);

}

HumanFollower::~HumanFollower() {

}

HumanFollower* HumanFollower::getInstance() {
	if (instance == NULL)
		instance = new HumanFollower();
	return instance;
}

void HumanFollower::navigate() {

	ros::Time current_time;
	ros::Rate loop_rate(10);
	while (ros::ok()) {

		geometry_msgs::Twist msg;

		current_time = ros::Time::now();

		if (mHumanPositionX > MIN_DISTANCE
				&& mHumanPositionX > 0
				&& (current_time.sec - mLastDetectionTime.sec < MAX_DETECTION_TIME_GAP)) {

			msg.linear.x = MAX_LINEAR_VELOCITY;

			if (mHumanPositionY < 0) {
				msg.angular.z = MAX_ANGULAR_VELOCITY;
			} else {
				msg.angular.z = -MAX_ANGULAR_VELOCITY;
			}

		} else {
			msg.linear.x = 0;
			msg.angular.z = 0;
		}

	    ROS_INFO("Setting velocity: linear %f, angular %f", msg.linear.x, msg.angular.z);

	    mPubRobotVelocity.publish(msg);

	    ros::spinOnce();

	    loop_rate.sleep();
	}

}

void HumanFollower::odometryCallback(const nav_msgs::Odometry& msg)
{
	HumanFollower::getInstance()->mRobotPositionX = msg.pose.pose.position.x;
	HumanFollower::getInstance()->mRobotPositionY = msg.pose.pose.position.y;
	//ROS_INFO("Robot position: [%f, %f]", msg.pose.pose.position.x, msg.pose.pose.position.y);
}

void HumanFollower::humanpositionCallback(const tf::tfMessage& msg)
{
	HumanFollower::getInstance()->mLastDetectionTime = ros::Time::now();
	HumanFollower::getInstance()->mHumanPositionX = msg.transforms[0].transform.translation.x;
	HumanFollower::getInstance()->mHumanPositionY = msg.transforms[0].transform.translation.y;
	//ROS_INFO("Human position: [%f, %f]", msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y);
}

//void HumanFollower::depthCallback(const sensor_msgs::Image& msg)
//{
//	uint* depth = (uint*)msg.data[0];
//	depth = depth + (int)msg.step*msg.height/2;
//	HumanFollower::getInstance()->mDistance = *depth;
//	ROS_INFO("Got depth data: %d", *depth);
//}
