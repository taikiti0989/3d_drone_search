#include "ManualControl.h"
#include "Patroller.h"
#include "StateTracker.h"
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>

Patroller* patroller;

Patroller::Patroller() {}

Patroller::~Patroller() {
	destroy();
}

void Patroller::destroy() {}

void Patroller::checkState() {
	// switch statement in case we need to check signage
	// ROS_INFO( "OFF: x: %f Y: %f", fabs(stats->getOdom()->pose.pose.position.x - current_state.start.x), fabs(stats->getOdom()->pose.pose.position.y - current_state.start.y) );
	switch(current_state.direction) {
	case FORWARD:
	case BACKWARD:
		if(fabs(stats->getOdom()->pose.pose.position.x - current_state.start.x) >= current_state.distance) nextState();
		break;

	case RIGHT:
	case LEFT:
		if(fabs(stats->getOdom()->pose.pose.position.y - current_state.start.y) >= current_state.distance) nextState();
		break;
	}
}

void Patroller::nextState() {
	current_state.start = stats->getOdom()->pose.pose.position;

	if(current_state.direction == RIGHT || current_state.direction == LEFT) current_state.distance = current_state.distance + spacing;
	else current_state.distance = current_state.distance;
	current_state.direction = (current_state.direction + 1) % 4;

	if(current_state.direction == BACKWARD && current_state.distance > radius) patrolling = false;

	// ROS_INFO("SWITCHING TO STATE: DIR: %d DIS: %f", current_state.direction, current_state.distance);
}

// simple reflex function that responds to current altitude of bebop and state information
void Patroller::patrol() {
	if(!patrolling) return;

	double off = 0;

	// ROS_INFO("PATROLLING WITH STATE: DIR: %d DIS: %f", current_state.direction, current_state.distance);

	checkState();
	geometry_msgs::Twist vel;
	speed = control->getSpeed();
	switch(current_state.direction) {
	case FORWARD:

		// ROS_INFO("EXECUTING FORWARD");
		vel.linear.x = speed;
		off = (stats->getOdom()->pose.pose.position.y - current_state.start.y);
		vel.linear.y = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

	case RIGHT:

		// ROS_INFO("EXECUTING RIGHT");
		vel.linear.y = -speed;
		off = (stats->getOdom()->pose.pose.position.x - current_state.start.x);
		vel.linear.x = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

	case BACKWARD:

		// ROS_INFO("EXECUTING BACKWARD");
		vel.linear.x = -speed;
		off = (stats->getOdom()->pose.pose.position.y - current_state.start.y);
		vel.linear.y = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;

	case LEFT:

		// ROS_INFO("EXECUTING LEFT");
		vel.linear.y = speed;
		off = (stats->getOdom()->pose.pose.position.x - current_state.start.x);
		vel.linear.x = (off > 0) ? fmin(off, speed) : fmax(off, -speed);
		control->send(&vel);
		break;
	}

	// ROS_INFO("SENT X: %f Y: %f", vel.linear.x, vel.linear.y);
}

void Patroller::stop() {
	ROS_INFO("STOPPING PATROL");
	patrolling = false;
	if( !control->isEnabled() ) control->toggle();
}

void Patroller::start(double altitude, double spacing) {
	ROS_INFO("STARTING PATROL");
	if( control->isEnabled() ) control->toggle();
	radius = (int) ceil(altitude);
	this->spacing = spacing;
	patrolling = true;

	current_state.distance = spacing;
	current_state.direction = FORWARD;
	current_state.start = stats->getOdom()->pose.pose.position;
}
void Patroller::goToPose(geometry_msgs::PoseStamped goalPose, geometry_msgs::PoseStamped curPose, double goalYaw, double curYaw) {
	
	
	if( !control->isEnabled() ) control->toggle();
	double target_x, target_y, target_z, dif, ab_y, ab_x, ab_z, target_theta;
	double off1=0;
	double off2 =0;
	double dif_yaw = 0;
	double V=0.06;
	geometry_msgs::Twist vel;
	bool stop = false;
	speed = control->getSpeed();
	// ROS_INFO("curPose.pose.position.x, curPose.pose.position.y = %f,%f", curPose.pose.position.x,curPose.pose.position.y);
	target_x = goalPose.pose.position.x - curPose.pose.position.x;
	target_y = goalPose.pose.position.y - curPose.pose.position.y;
	target_z = 0;
	if (goalYaw != 0)
	{	speed = 0.1;
		dif_yaw = fabs(goalYaw) - fabs(curYaw);
		
	}
	
	goal_reached = false;

		ab_x = fabs(target_x);
		ab_y = fabs(target_y);
		dif = sqrt(target_x * target_x + target_y * target_y+ target_z * target_z);
		target_theta = atan2(ab_y,ab_x);
		// ROS_INFO(" dif = %f", dif);
		// ROS_INFO("tar_x, tar_y = %f, %f,", target_x, target_y);
		// double a = target_theta - stats->slam_yaw;
		if(target_x<0 && target_y>0){
			double a = target_theta; //- slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			// ROS_INFO("1");
		}
		//////2////////
		if(target_x>0 && target_y>0){
			double a = target_theta; // + slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = V*dif*sin(a);
			// ROS_INFO("2");
		}
		//////3////////
		if(target_x>0 && target_y<0){
			double a = target_theta; // - slam_roll;
			vel.linear.x = V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			// ROS_INFO("3");
		}
		if(target_x<0 && target_y<0){
			double a = target_theta; /// + slam_roll;
			vel.linear.x = -V*dif*cos(a);
			vel.linear.y = -V*dif*sin(a);
			// ROS_INFO("4");
		}

		if(fabs(dif) < 0.2 && fabs(dif_yaw) < 0.3) {
			ROS_INFO("STOP ...");
			vel.linear.x = 0;
			vel.linear.y = 0;
			vel.linear.z = 0;
			vel.angular.z = 0;
			control->send(&vel);
			k++;
			
			if(k >10) {
				
				goal_reached=true;
				if( control->isEnabled() ) control->toggle();
				k = 0;
				return;
				
			}
		}
		
	if (dif > 0.2 )
	{
		speed = 0.06;
		vel.linear.x = (vel.linear.x > 0) ? fmin(vel.linear.x, speed) : fmax(vel.linear.x, -speed);
		vel.linear.y = (vel.linear.y > 0) ? fmin(vel.linear.y, speed) : fmax(vel.linear.y, -speed);
		ROS_INFO(" vel.x, vel.y = %f, %f", vel.linear.x, vel.linear.y);
		control->send(&vel);
	}
	if (dif < 0.2 && fabs(dif_yaw) > 0.3)
	{
		
			speed = 0.1;
			vel.angular.z = (dif_yaw > 0) ? fmin(dif_yaw, speed) : fmax(dif_yaw, -speed);
		
			
		control->send(&vel);
		ROS_INFO("diff yaw = %f", dif_yaw);
		ROS_INFO("vel.z = %f", vel.angular.z);
	}
	
	
}
