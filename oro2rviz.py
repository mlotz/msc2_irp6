#!/usr/bin/env python

import rospy
from irpos import *
from exirpos import *
import numpy as np
from std_msgs.msg import *
from visualization_msgs.msg import Marker
from math import *
from geometry_msgs.msg import *
from tf.transformations import *
# DEMO FUNCTIONS

# POSTUMENT DEMOS


def setMarker(nrId,c, reactionForceAngle, vectorAngle, givenForceAngle):
#def setMarker(nrId,c):
	
	#print('asasdsda')

	current = irpos.get_cartesian_pose()
	#marker = Marker()
	

	marker.header.frame_id = "/pl_base"
	#marker.header.stamp = rospy.get_rostime() + rospy.Duration(0.002)
	marker.ns = "basic_shapes"
	marker.id = nrId+10
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	#marker.pose.position.x = 1.0
	#marker.pose.position.y = 0.0
	#marker.pose.position.z = 1.0
	marker.pose.position.x = current.position.x
	marker.pose.position.y = current.position.y
	marker.pose.position.z = current.position.z 
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = 0.003
	marker.scale.y = 0.003
	marker.scale.z = 0.003
	marker.color.r = 0.0
	marker.color.g = c
	marker.color.b = 0.0
	marker.color.a = 1.0
	#marker.lifetime = rospy.Duration(10.0)
	
	#print(str(rviz_pub.getNumSubscribers()))
	
	rviz_pub.publish(marker)
	
	#arrow
	marker.scale.x = 0.05
	marker.scale.y = 0.002
	marker.scale.z = 0.002
	marker.id = 0
	marker.type = marker.ARROW
	
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = np.sin(reactionForceAngle/2)
	marker.pose.orientation.w = np.cos(reactionForceAngle/2)
	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	rviz_pub.publish(marker)
	
	#arrow
	marker.scale.x = 0.05
	marker.scale.y = 0.002
	marker.scale.z = 0.002
	marker.id = 1
	marker.type = marker.ARROW
	
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = np.sin(vectorAngle/2)
	marker.pose.orientation.w = np.cos(vectorAngle/2)
	marker.color.r = 0.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	rviz_pub.publish(marker)

	#arrow
	marker.scale.x = 0.05
	marker.scale.y = 0.002
	marker.scale.z = 0.002
	marker.id = 2
	marker.type = marker.ARROW
	
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = np.sin(givenForceAngle/2)
	marker.pose.orientation.w = np.cos(givenForceAngle/2)
	marker.color.r = 0.0
	marker.color.g = 0.0
	marker.color.b = 1.0
	marker.color.a = 1.0
	rviz_pub.publish(marker)
def delMarker(nrId):
	marker.id = nrId
	marker.action = marker.DELETE
	rviz_pub.publish(marker)
def setArrows(nrId,c, reactionForceAngle, vectorAngle, givenForceAngle):
#def setMarker(nrId,c):
	
	#print('asasdsda')

	current = irpos.get_cartesian_pose()
	#marker = Marker()
	

	marker.header.frame_id = "/pl_base"
	#marker.header.stamp = rospy.get_rostime() + rospy.Duration(0.002)
	marker.ns = "basic_shapes"
	marker.id = nrId+10
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	#marker.pose.position.x = 1.0
	#marker.pose.position.y = 0.0
	#marker.pose.position.z = 1.0
	marker.pose.position.x = current.position.x
	marker.pose.position.y = current.position.y
	marker.pose.position.z = current.position.z 
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = 0.003
	marker.scale.y = 0.003
	marker.scale.z = 0.003
	marker.color.r = 0.0
	marker.color.g = c
	marker.color.b = 0.0
	marker.color.a = 1.0
	#marker.lifetime = rospy.Duration(10.0)
	
	#print(str(rviz_pub.getNumSubscribers()))
	
	
	#arrow
	marker.scale.x = 0.05
	marker.scale.y = 0.002
	marker.scale.z = 0.002
	marker.id = 0
	marker.type = marker.ARROW
	
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = np.sin(reactionForceAngle/2)
	marker.pose.orientation.w = np.cos(reactionForceAngle/2)
	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	rviz_pub.publish(marker)
	
	#arrow
	marker.scale.x = 0.05
	marker.scale.y = 0.002
	marker.scale.z = 0.002
	marker.id = 1
	marker.type = marker.ARROW
	
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = np.sin(vectorAngle/2)
	marker.pose.orientation.w = np.cos(vectorAngle/2)
	marker.color.r = 0.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	rviz_pub.publish(marker)

	#arrow
	marker.scale.x = 0.05
	marker.scale.y = 0.002
	marker.scale.z = 0.002
	marker.id = 2
	marker.type = marker.ARROW
	
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = np.sin(givenForceAngle/2)
	marker.pose.orientation.w = np.cos(givenForceAngle/2)
	marker.color.r = 0.0
	marker.color.g = 0.0
	marker.color.b = 1.0
	marker.color.a = 1.0
	rviz_pub.publish(marker)
def setDot(nrId,c, reactionForceAngle, vectorAngle, givenForceAngle):
#def setMarker(nrId,c):
	
	#print('asasdsda')

	current = irpos.get_cartesian_pose()
	#marker = Marker()
	

	marker.header.frame_id = "/pl_base"
	#marker.header.stamp = rospy.get_rostime() + rospy.Duration(0.002)
	marker.ns = "basic_shapes"
	marker.id = nrId+10
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	#marker.pose.position.x = 1.0
	#marker.pose.position.y = 0.0
	#marker.pose.position.z = 1.0
	marker.pose.position.x = current.position.x
	marker.pose.position.y = current.position.y
	marker.pose.position.z = current.position.z 
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = 0.003
	marker.scale.y = 0.003
	marker.scale.z = 0.003
	marker.color.r = 0.0
	marker.color.g = c
	marker.color.b = 0.0
	marker.color.a = 1.0
	#marker.lifetime = rospy.Duration(10.0)
	
	#print(str(rviz_pub.getNumSubscribers()))
	
	rviz_pub.publish(marker)
	

def rvizzz():
	rviz_pub.publish(marker)

def setArrow(pose, id, color_r,color_g,color_b):
	marker.header.frame_id = "/pl_base"

	marker.ns = "basic_shapes"
	marker.type = marker.ARROW
	marker.action = marker.ADD

	marker.scale.x = 0.5
	marker.scale.y = 0.02
	marker.scale.z = 0.02
	marker.id = id
	
	marker.pose = pose
	marker.color.r = color_r
	marker.color.g = color_g
	marker.color.b = color_b
	marker.color.a = 1.0
	rviz_pub.publish(marker)


def Tholder():
	
	#print "Irp6ot: Behavior: arrows."

	current = irpos.get_cartesian_pose()
	currentSensorPose = irpos.get_SensorPose();
	#print currentSensorPose
	#print currentWristOutputPose
	origin,xaxis,yaxis,zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

	setArrow(currentSensorPose,11,1.0,0.0,0.0);
	q1 = [currentSensorPose.orientation.x,currentSensorPose.orientation.y,currentSensorPose.orientation.z,currentSensorPose.orientation.w] 
	qt1=quaternion_about_axis(-pi/2, yaxis)
	q2=quaternion_multiply(q1, qt1)
	yAxisPose = currentSensorPose
	yAxisPose.orientation.x = q2[0]
	yAxisPose.orientation.y = q2[1]
	yAxisPose.orientation.z = q2[2]
	yAxisPose.orientation.w = q2[3]
	setArrow(yAxisPose, 12, 0.0,0.0,1.0)

	qt2=quaternion_about_axis(pi/2, zaxis)
	q3=quaternion_multiply(q1, qt2)
	zAxisPose = currentSensorPose
	zAxisPose.orientation.x = q3[0]
	zAxisPose.orientation.y = q3[1]
	zAxisPose.orientation.z = q3[2]
	zAxisPose.orientation.w = q3[3]
	setArrow(zAxisPose, 13, 0.0,1.0,0.0)


	#print "done"

def Ttest():
	irpos = exirpos("IRpOS", "Irp6p", 6, "irp6p_manager")
	print "Irp6ot: Behavior: arrows."

	current = irpos.get_HandForce2()
	print current



#MAIN
irpos = exirpos("IRpOS", "Irp6p", 6, "irp6p_manager")
rviz_pub = rospy.Publisher('visualization_marker', Marker)
marker = Marker()
time.sleep(1.0)
#setMarker(0)
#while(1):
Tholder()
	#time.sleep(0.1);
