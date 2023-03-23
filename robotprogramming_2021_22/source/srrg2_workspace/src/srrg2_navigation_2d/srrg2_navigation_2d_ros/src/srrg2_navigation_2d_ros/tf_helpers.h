#pragma once
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

bool getTfTransform(Eigen::Isometry3f& dest,
                    tf::TransformListener& listener,
                    const std::string& reference,
                    const std::string& target,
                    const ros::Time& time);

bool getTfTransform(Eigen::Isometry2f& dest,
                    tf::TransformListener& listener,
                    const std::string& reference,
                    const std::string& target,
                    const ros::Time& time);

void getIsometry(Eigen::Isometry3f& dest, const geometry_msgs::Pose& src);

void getIsometry(Eigen::Isometry2f& dest, const geometry_msgs::Pose& src);
