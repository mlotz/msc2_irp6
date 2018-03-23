


#include "rvizMarkers.h"


bool setArrow(visualization_msgs::Marker& markerRef,geometry_msgs::Pose markerPose)
{
	//visualization_msgs::Marker marker;
	markerRef.header.frame_id = "/pl_base";
	//marker.header.stamp = ros::Time::now();
	markerRef.header.stamp = rtt_rosclock::host_now();
	markerRef.ns = "basic_shapes";
	markerRef.id = 0;
	markerRef.type = visualization_msgs::Marker::ARROW;
	markerRef.action = visualization_msgs::Marker::ADD;
	//marker.pose.position.x = 0;
	//marker.pose.position.y = 0;
	//marker.pose.position.z = 0;
	//marker.pose.orientation.x = 0.0;
	//marker.pose.orientation.y = 0.0;
	//marker.pose.orientation.z = 0.0;
	//marker.pose.orientation.w = 1.0;
	markerRef.pose = markerPose;

	markerRef.scale.x = 0.5;
	markerRef.scale.y = 0.02;
	markerRef.scale.z = 0.02;
	markerRef.color.r = 0.0;
	markerRef.color.g = 1.0;
	markerRef.color.b = 0.0;
	markerRef.color.a = 2.0;
	markerRef.lifetime = ros::Duration();
	
	return true;
}
