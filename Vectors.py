#!/usr/bin/env python

import rospy
from irpos import *
from exirpos import *
import numpy as np
from std_msgs.msg import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import *
from tf.transformations import *

# DEMO FUNCTIONS

# POSTUMENT DEMOS


	
def T2():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: T2 - Starting."
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 4.7123889804 , 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	
	print "Irp6p: Behavior: T2 - Topdown."



def T3():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: T3 - Starting."
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 3.14159265359 , 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	
	print "Irp6p: Behavior: T3 - Left."
	



def T4():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: T4 - Starting."
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 6.28318530718 , 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	
	print "Irp6p: Behavior: T4 - Left."
def TposA():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: T3 - Starting."
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 3.34159265359 , -1.4907963268], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	
	print "Irp6p: Behavior: T3 - Left."

def TposB():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: TposB - Starting."
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 3.34159265359 , 1.65079632679], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	
	print "Irp6p: Behavior: T3 - Left."

def TposC():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: T2 - Starting."
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 4.8123889804 , 1.65079632679], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	
	print "Irp6p: Behavior: TposC - Topdown."

def TgetWeightsABC():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: getWeights - Experiment"
	TposA()
	time.sleep(10.0)
	TposB()
	time.sleep(10.0)
	TposC()
	time.sleep(10.0)

	print "Done"
	
	
def T5():
	#irpos = IRPOS("IRpOS", "Irp6ot", 7 , "irp6ot_manager")
	irpos = IRPOS("IRpOS", "Irp6ot", 7)
	print "Irp6ot: Behavior: T5 - Rise!."
	
	
	irpos.move_rel_to_cartesian_pose(15.0, Pose(Point(0.0, 0.0, -0.15), Quaternion(0.0, 0.0, 0.0, 1.0)))
	#print str(irpos.get_tfg_joint_position())

	print "Irp6ot: Behavior: T5 - done."

	
def T6():
	#irpos = IRPOS("IRpOS", "Irp6ot", 7 , "irp6ot_manager")
	#irpos = IRPOS("IRpOS", "Irp6ot", 7)
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
	print "Irp6ot: Behavior: T6 - Grabbing."
	# grabbed 0.061
	# dropped 0.065
	irpos.tfg_to_joint_position(0.061, 10.0)
	
	print str(irpos.get_tfg_joint_position())

	print "Irp6ot: Behavior: T6 - done."

def T7():
	#irpos = IRPOS("IRpOS", "Irp6ot", 7 , "irp6ot_manager")
	#irpos = IRPOS("IRpOS", "Irp6ot", 7)
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
	print "Irp6ot: Behavior: T7 - Dropping."
	# grabbed 0.061
	# dropped 0.065
	irpos.tfg_to_joint_position(0.065, 10.0)
	
	print str(irpos.get_tfg_joint_position())

	print "Irp6ot: Behavior: T7 - done."


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



def Tholder():
	#irpos = IRPOS("IRpOS", "Irp6ot", 7 , "irp6ot_manager")
	#irpos = IRPOS("IRpOS", "Irp6ot", 7)
	#irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
	irpos = exirpos("IRpOS", "Irp6p", 6, "irp6p_manager")
	print "Irp6ot: Behavior: arrows."

	current = irpos.get_cartesian_pose()
	
	currentWristOutputPose = irpos.get_WristOutputPose_cartesian_pose()
	#print current

	print currentWristOutputPose
	#marker = Marker()
	

	marker.header.frame_id = "/pl_base"
	#marker.header.stamp = rospy.get_rostime() + rospy.Duration(0.002)
	marker.ns = "basic_shapes"
	marker.id = 11
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	#marker.pose.position.x = 1.0
	#marker.pose.position.y = 0.0
	#marker.pose.position.z = 1.0


	marker.pose.position.x = currentWristOutputPose.position.x
	marker.pose.position.y = currentWristOutputPose.position.y
	marker.pose.position.z = currentWristOutputPose.position.z 
	origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
	q1 = [currentWristOutputPose.orientation.x, currentWristOutputPose.orientation.y, currentWristOutputPose.orientation.z, currentWristOutputPose.orientation.w]
	qt1 = quaternion_about_axis(-1.57079632679, yaxis)
	qt2 = quaternion_about_axis(-1.57079632679, zaxis)
	newRotation = quaternion_multiply(q1, qt1)
	newRotation = quaternion_multiply(newRotation, qt2)
	print newRotation
	
	marker.pose.orientation.x = newRotation[0]
	marker.pose.orientation.y = newRotation[1]
	marker.pose.orientation.z = newRotation[2]
	marker.pose.orientation.w = newRotation[3]

	#ret
	#marker.pose.position.x = currentWristOutputPose.position.x
	#marker.pose.position.y = currentWristOutputPose.position.y
	#marker.pose.position.z = currentWristOutputPose.position.z 
	#marker.pose.orientation.x = currentWristOutputPose.orientation.x
	#marker.pose.orientation.y = currentWristOutputPose.orientation.y
	#marker.pose.orientation.z = currentWristOutputPose.orientation.z
	#marker.pose.orientation.w = currentWristOutputPose.orientation.w
	marker.scale.x = 0.003
	marker.scale.y = 0.003
	marker.scale.z = 0.003
	marker.color.r = 0.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.color.a = 1.0
	#marker.lifetime = rospy.Duration(10.0)
	
	#print(str(rviz_pub.getNumSubscribers()))
	
	
	#arrow
	marker.scale.x = 0.5
	marker.scale.y = 0.02
	marker.scale.z = 0.02
	marker.id = 0
	marker.type = marker.ARROW
	
	marker.pose.orientation.x = newRotation[0]
	marker.pose.orientation.y = newRotation[1]
	marker.pose.orientation.z = newRotation[2]
	marker.pose.orientation.w = newRotation[3]

	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.color.a = 2.0
	rviz_pub.publish(marker)



	print "done"

#MAIN
irpos = exirpos("IRpOS", "Irp6p", 6, "irp6p_manager")
rviz_pub = rospy.Publisher('visualization_marker', Marker)
marker = Marker()
#setMarker(0)
time.sleep(1.0)
if __name__ == '__main__':
	if sys.argv[1]=="p_m":
		irp6p_multi_trajectory()	
	elif sys.argv[1]=="p_m2":
		irp6p_multi_trajectory2()	
	elif sys.argv[1]=="p_s":
		irp6p_get_status()
	elif sys.argv[1]=="p_sp":
		irp6p_synchro_position()
	elif sys.argv[1]=="ot_m":
		irp6otm_multi_trajectory()	
	elif sys.argv[1]=="ot_m2":
		irp6otm_multi_trajectory2()	
	elif sys.argv[1]=="ot_s":
		irp6otm_get_status()	
	elif sys.argv[1]=="ot_sp":
		irp6otm_synchro_position()
	elif sys.argv[1]=="test":
		test()
	elif sys.argv[1]=="T1":
		T1()
	elif sys.argv[1]=="T2":
		T2()
	elif sys.argv[1]=="T3":
		T3()
	elif sys.argv[1]=="T4":
		T4()
	elif sys.argv[1]=="T5":
		T5()
	elif sys.argv[1]=="T6":
		T6()
	elif sys.argv[1]=="T7":
		T7()
	elif sys.argv[1]=="TS":
		TSynch()
	elif sys.argv[1]=="TC":
		TC()
	elif sys.argv[1]=="TR":
		TRacking()
	elif sys.argv[1]=="TR2":
		TRacking2()
	elif sys.argv[1]=="TR3":
		TRacking3()
	elif sys.argv[1]=="RV":
		rvizzz()
	elif sys.argv[1]=="TCo":
		ToContact()
	elif sys.argv[1]=="TL":
		TestLogging()
	elif sys.argv[1]=="TposA":
		TposA()
	elif sys.argv[1]=="TposB":
		TposB()
	elif sys.argv[1]=="TposC":
		TposC()
	elif sys.argv[1]=="TgetWeightsABC":
		TgetWeightsABC()
	elif sys.argv[1]=="Tholder":
		Tholder()
