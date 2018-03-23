#ifndef RVIZMARKERS_H_
#define RVIZMARKERS_H_

#include <visualization_msgs/Marker.h>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include "rtt_rosclock/rtt_rosclock.h"

bool setArrow(visualization_msgs::Marker& markerRef, geometry_msgs::Pose markerPose);

		

#endif // RVIZMARKERS_H_
