#ifndef BEBOP_TELEOP_BEBOP_TELEOP_H
#define BEBOP_TELEOP_BEBOP_TELEOP_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

extern const std::string tracking;
extern const std::string darknet_Boxes_Topic;

class BebopTrack
{
public:
    BebopTrack();
    ~BebopTrack();
    bool _isTracking;

    void _boxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &boxMessage);
    void Bebop_tracking();
    explicit BebopTrack(const ros::NodeHandle& nodeHandle);
};

#endif //BEBOP_TELEOP_BEBOP_TELEOP_H
