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

def irp6p_multi_trajectory():
	irpos = IRPOS("IRpOS", "Irp6p", 6)

	motor_trajectory = [JointTrajectoryPoint([0.4, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0)), JointTrajectoryPoint([10.0, 10.0, 0.0, 10.57, 10.57, -20.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(12.0))]
        irpos.move_along_motor_trajectory(motor_trajectory)

	joint_trajectory = [JointTrajectoryPoint([0.4, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(3.0)),JointTrajectoryPoint([0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(6.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)

	rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.18029263241))
	rot2 = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.3, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
	cartesianTrajectory = [CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()), CartesianTrajectoryPoint(rospy.Duration(6.0), pm.toMsg(rot), Twist()),CartesianTrajectoryPoint(rospy.Duration(9.0), pm.toMsg(rot2), Twist())]
	irpos.move_along_cartesian_trajectory(cartesianTrajectory)

	toolParams = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
	irpos.set_tool_geometry_params(toolParams)

	rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
	cartesianTrajectory = [CartesianTrajectoryPoint(rospy.Duration(3.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()),
CartesianTrajectoryPoint(rospy.Duration(6.0), pm.toMsg(rot), Twist()),CartesianTrajectoryPoint(rospy.Duration(9.0), Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.63691, 0.096783, 0.75634, -0.11369)), Twist())]
	irpos.move_along_cartesian_trajectory(cartesianTrajectory)

	toolParams = Pose(Point(0.0, 0.0, 0.25), Quaternion(0.0, 0.0, 0.0, 1.0))
	irpos.set_tool_geometry_params(toolParams)

	print "Irp6p 'multi_trajectory' test compleated"


def T1():
	irpos = IRPOS("IRpOS", "Irp6p", 6)

	
	cartesianTrajectory = [CartesianTrajectoryPoint(rospy.Duration(6.0), Pose(Point(0.8, 0.0, 1.3), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()) , CartesianTrajectoryPoint(rospy.Duration(12.0), Pose(Point(0.8, 0.1, 1.3), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()) , CartesianTrajectoryPoint(rospy.Duration(18.0), Pose(Point(0.8, 0.1, 1.4), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()) , CartesianTrajectoryPoint(rospy.Duration(24.0), Pose(Point(0.8, 0.0, 1.4), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist()) , CartesianTrajectoryPoint(rospy.Duration(30.0), Pose(Point(0.8, 0.0, 1.3), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)), Twist())  ]
	irpos.move_along_cartesian_trajectory(cartesianTrajectory)


	print "Irp6p: Behavior: T1 - done."
	
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
def TposB():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: T3 - Starting."
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 3.34159265359 , -1.4907963268], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	
	print "Irp6p: Behavior: T3 - Left."

def TposA():
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
def TposGrab():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: T2 - Starting."
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 4.8123889804 , 1.65079632679], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	irpos.tfg_to_joint_position(0.074, 5.0)
	irpos.move_rel_to_cartesian_pose(10.0, Pose(Point(0.0, 0.0, 0.15), Quaternion(0.0, 0.0, 0.0, 1.0)))
	print "Irp6p: Behavior: TposC - Topdown."
def TposD():
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")

	print "Irp6p: Behavior: T2 - Starting."
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 4.8123889804 , 1.65079632679-pi], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(10.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	
	print "Irp6p: Behavior: TposC - Topdown."

def TgetWeightsABC():
	irpos = exirpos("IRpOS", "Irp6p", 6, "irp6p_manager")


	print "Irp6p: Behavior: getWeights - Experiment"
	TposA()
	HandForce1_bias = irpos.get_HandForce1().vector.z
	TposB()
	HandForce1_fb = irpos.get_HandForce1()
	HandForce1_w = (HandForce1_fb.vector.z-HandForce1_bias) / 2.0
	WGPx = -HandForce1_fb.vector.y / (2*HandForce1_w)
	WGPy = HandForce1_fb.vector.x / (2*HandForce1_w)
	print HandForce1_w
	print WGPx
	print WGPy
	TposC()
	HandForce1_fc = irpos.get_HandForce1()
	WGPz = (HandForce1_fc.vector.y/HandForce1_w)-(HandForce1_fb.vector.y/(2*HandForce1_w))
	print WGPz
	#print irpos.get_HandForce1()
	#TposD()
	#print irpos.get_HandForce1()

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
	irpos.tfg_to_joint_position(0.074, 5.0)
	
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

def TC():
	#irpos = IRPOS("IRpOS", "Irp6ot", 7 , "irp6ot_manager")
	irpos = IRPOS("IRpOS", "Irp6ot", 7)
	print "Irp6ot: Behavior: TC - decendt to contact."
	
	irpos.move_to_synchro_position(10.0)
	joint_trajectory = [JointTrajectoryPoint([0.0,0.0, -1.5707963268, 0.0, 0.0, 4.7123889804 , 0.0], [0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(15.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	#irpos.move_rel_to_cartesian_pose(10.0, Pose(Point(0.0, 0.0, -0.03), Quaternion(0.0, 0.0, 0.0, 1.0)))	
	irpos.move_rel_to_cartesian_pose_with_contact( 20.0, Pose(Point(0.0, 0.0, 0.20), Quaternion(0.0, 0.0, 0.0, 1.0)),Wrench(Vector3(10.0, 10.0, 4.0), Vector3(0.0, 0.0, 0.0)))
	irpos.move_rel_to_cartesian_pose(2.0, Pose(Point(0.0, 0.0, -0.01), Quaternion(0.0, 0.0, 0.0, 1.0)))
	#irpos.move_rel_to_cartesian_pose(2.0, Pose(Point(0.01, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
	irpos.move_rel_to_cartesian_pose_with_contact( 20.0, Pose(Point(0.20, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),Wrench(Vector3(6.0, 6.0, 6.0), Vector3(0.0, 0.0, 0.0)))
	#time.sleep(6.0)
	P1 = irpos.get_cartesian_pose()
	print str(P1)
	P1x = P1.position.x
	P1y = P1.position.y
	P1z = P1.position.z
	time.sleep(1.0)
	irpos.move_rel_to_cartesian_pose_with_contact( 30.0, Pose(Point(-0.30, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),Wrench(Vector3(8.0, 8.0, 8.0), Vector3(0.0, 0.0, 0.0)))
	P2 = irpos.get_cartesian_pose()
	print str(P2)
	P2x = P2.position.x
	P2y = P2.position.y
	P2z = P2.position.z
	time.sleep(1.0)
	irpos.move_rel_to_cartesian_pose_with_contact( 20.0, Pose(Point(0.0, -0.20, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),Wrench(Vector3(6.0, 6.0, 6.0), Vector3(0.0, 0.0, 0.0)))
	P3 = irpos.get_cartesian_pose()
	print str(P3)
	P3x = P3.position.x
	P3y = P3.position.y
	P3z = P3.position.z
	time.sleep(1.0)
	
	A = np.array([P1x, P1y, P1z])
	B = np.array([P2x, P2y, P2z])
	C = np.array([P3x, P3y, P3z])
	a = np.linalg.norm(C - B)
	b = np.linalg.norm(C - A)
	c = np.linalg.norm(B - A)
	s = (a + b + c) / 2
	R = a*b*c / 4 / np.sqrt(s * (s - a) * (s - b) * (s - c))
	b1 = a*a * (b*b + c*c - a*a)
	b2 = b*b * (a*a + c*c - b*b)
	b3 = c*c * (a*a + b*b - c*c)
	P = np.column_stack((A, B, C)).dot(np.hstack((b1, b2, b3)))
	P /= b1 + b2 + b3
	
	
	print P
	print P[0]
	T = P - C 
	print T
	time.sleep(1.0)
	
	irpos.move_rel_to_cartesian_pose( 20.0, Pose(Point(T[1],T[0] , 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
	time.sleep(10.0)
	#move_to_cartesian_pose(20.0, Pose(P1))
	#irpos.move_rel_to_cartesian_pose(6.0, Pose(Point(-0.08, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
	irpos.move_rel_to_cartesian_pose(15.0, Pose(Point(0.0, 0.0, -0.14), Quaternion(0.0, 0.0, 0.0, 1.0)))
	irpos.move_to_synchro_position(10.0)
	
	print "Irp6ot: Behavior: TC - done. Current position at center"
def ToContact():
	print "Irp6ot: Behavior: To contact - Synchronizing."
	#irpos.move_to_synchro_position(15.0)
	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 4.7123889804 , 0.0], [0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(15.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)

	irpos.move_rel_to_cartesian_pose_with_contact( 20.0, Pose(Point(0.0, 0.0, 0.20), Quaternion(0.0, 0.0, 0.0, 1.0)),Wrench(Vector3(10.0, 10.0, 7.0), Vector3(0.0, 0.0, 0.0)))
	irpos.move_rel_to_cartesian_pose(2.0, Pose(Point(0.0, 0.0, -0.007), Quaternion(0.0, 0.0, 0.0, 1.0)))

	irpos.move_rel_to_cartesian_pose_with_contact( 20.0, Pose(Point(0.20, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),Wrench(Vector3(6.0, 6.0, 0.0), Vector3(0.0, 0.0, 0.0)))
	irpos.move_rel_to_cartesian_pose(4.0, Pose(Point(-0.01, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
	irpos.EX_print_wrench()
	print "Irp6ot: Behavior: To contact - done."
		

def TSynch():
	irpos = IRPOS("IRpOS", "Irp6p", 6 , "irp6p_manager")
	#irpos = IRPOS("IRpOS", "Irp6ot", 7)
	print "Irp6ot: Behavior: TS - Synchronizing."
	irpos.move_to_synchro_position(10.0)

	print "Irp6ot: Behavior: TS - done."

def TestLogging():
	irpos.start_logging()
	time.sleep(5.0)
	irpos.stop_logging()

def TRacking3():

	print "Irp6p: Behavior: TRacking3 - Starting."
	
	#T2()
	ToContact()

	time.sleep(1.0)
	
	angle = 0
	counter = 0
	once = 1
	reactionForceAngle = np.pi/2
	#step = 0.001
	nominalSpeed = 0.008
	#nominalSpeed = 0.015 nope
	speed =0.000
	#speed =0.005
	nominalContactForce=4.0
	contactForce= 4.0
	#speed =0.0
	#contactForce= 0.0
	zVConstraint = 0.005
	#forcecontrol 
	weight = 10.8
  	mass_center = Vector3(0.004, 0.0, 0.156)
	irpos.set_tool_physical_params( weight , mass_center)
	inertia = Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.5, 0.5, 0.5))
  	reciprocaldamping = ReciprocalDamping(Vector3(0.0012, 0.0012, 0.0), Vector3(0.0, 0.0, 0.0))
  	#wrench = Wrench(Vector3(np.sin(angle+(np.pi/2))*contactForce, np.cos(angle+(np.pi/2))*contactForce, 0.0), Vector3(0.0, 0.0, 0.0))
	wrench = Wrench(Vector3(contactForce, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
	#wrench = Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
  	twist = Twist(Vector3(0.0, -speed, 0.0), Vector3(0.0, 0.0, 0.0))
	print('[Cartesian pose]')
	current = irpos.get_cartesian_pose()
	print '[OverScript] print Cartesian Pose'
	print str(current)
	zPlane = current.position.z
	zV = 0.0
	maxzDelta = 0.0
	time.sleep(1.0)
	irpos.start_force_controller(inertia, reciprocaldamping, wrench, twist)
	time.sleep(0.1)
	irpos.start_logging()
	time.sleep(0.1)
	#for iter in range(0,719):
	#for iter in range(0,60000):
	for iter in range(0,60000):
	#while (1):
		time.sleep(0.01)
		#setMarker(iter,0.5)
		#angle = -iter * (np.pi/360)- np.pi/2;
		
		onWrench = irpos.get_force_readings()
		current = irpos.get_cartesian_pose()
		#onWrench.force.x = 1.0 * sin(angle)
		#onWrench.force.y = 1.0 * cos(angle)
		#onWrench.force.x = -1.0 
		#onWrench.force.y = 0.0 
		#onWrench.force.x = 0.0 
		#onWrench.force.y = -1.0 
		#print str(onWrench.force.x)
		Fxy = np.sqrt(onWrench.force.x*onWrench.force.x+onWrench.force.y*onWrench.force.y)
		sinAlfa = onWrench.force.y/Fxy
		cosAlfa = onWrench.force.x/Fxy
		#onWrench.force.x=-onWrench.force.x
		#onWrench.force.y=-onWrench.force.y
		#if(Fxy<1.0):
			#sinAlfa = 0.0
			#cosAlfa = -1.0
			
		#angle = np.arcsin(sinAlfa)
		#print 'Fxy ='+str(Fxy) 
		if(Fxy>2.0):
			if(Fxy>4.0):
				speed = nominalSpeed*((2*nominalContactForce)-Fxy)/nominalContactForce
				if (speed<0.00):
					speed = 0.00
			else:
				speed = nominalSpeed*(Fxy/nominalContactForce)
			reactionForceAngle=np.angle(onWrench.force.y+onWrench.force.x*(0+1j))
			#reactionForceAngle=np.angle(onWrench.force.x+onWrench.force.y*(0+1j))
			if (once == 1):
				once = 0
				end = current
				print '[OverScript] Setting end condition to'
				print str(end)
		else:
			speed = 0.0

		#vectorAngle = reactionForceAngle + np.pi/2
		vectorAngle = reactionForceAngle + np.pi/2
		#if (vectorAngle>2*np.pi):
			#vectorAngle = vectorAngle -
		#givenForceAngle = reactionForceAngle + np.pi
		givenForceAngle = reactionForceAngle
		#if(Fxy<2.0):
			#reactionForceAngle = np.pi/2
		#print 'Angles:reactionForceAngle='+str(np.degrees(reactionForceAngle))+'|Fxy='+str(Fxy)
		#print str(iter)+'--'+str(reactionForceAngle)+'vect:'+str(vectorAngle)+'x'+str(onWrench.force.x)+'y:'+str(onWrench.force.y)
		#if(iter%5==0):
		#setMarker(iter,0.5,reactionForceAngle, vectorAngle, givenForceAngle)
		setArrows(iter,0.5,reactionForceAngle, vectorAngle, givenForceAngle)
		if(iter%20==0):
			setDot(iter,0.5,reactionForceAngle, vectorAngle, givenForceAngle)
		#updateArrows()

		
		#irpos.seamless_move_rel_to_cartesian_pose(0.04, Pose(Point(np.sin(angle)*step, np.cos(angle)*step, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)))
		#wrench = Wrench(Vector3(np.sin(angle-(np.pi/2))*contactForce, np.cos(angle-(np.pi/2))*contactForce, 0.0), Vector3(0.0, 0.0, 0.0))
		#wrench = Wrench(Vector3((sinAlfa-np.pi/2)*contactForce, (cosAlfa-np.pi/2)*contactForce, 0.0), Vector3(0.0, 0.0, 0.0))
		##wrench = Wrench(Vector3(-cosAlfa*contactForce, sinAlfa*contactForce, 0.0), Vector3(0.0, 0.0, 0.0))
		#wrench = Wrench(Vector3(cosAlfa*contactForce, -sinAlfa*contactForce, 0.0), Vector3(0.0, 0.0, 0.0))
		#wrench = Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
		wrench = Wrench(Vector3(np.sin(givenForceAngle)*contactForce, np.cos(givenForceAngle)*contactForce, 0.0), Vector3(0.0, 0.0, 0.0))
		zDelta = zPlane - current.position.z
		if (zDelta > maxzDelta):
			maxzDelta = zDelta
		zV = -zDelta*0.5
		
		if (Fxy>10.0):
			print irpos.REDFAIL+'[ERROR] Fxy>'+str(10.0)+'! On iteration:'+str(iter)+irpos.ENDC
			break
		if (zV>zVConstraint):
			print irpos.REDFAIL+'[ERROR] zV>'+str(zVConstraint)+'! On iteration:'+str(iter)+irpos.ENDC
			break
		if (once!=1)and(np.sqrt((current.position.x - end.position.x)*(current.position.x - end.position.x)+(current.position.y - end.position.y)*(current.position.y - end.position.y))<0.008)and(iter>1000):
			print '[OverSript] End condition reached on iter ='+str(iter)
			break
		#twist = Twist(Vector3(np.sin(angle)*speed, np.cos(angle)*speed, zV), Vector3(0.0, 0.0, 0.0))
		#twist = Twist(Vector3(sinAlfa*speed, cosAlfa*speed, zV), Vector3(0.0, 0.0, 0.0))
		twist = Twist(Vector3(np.sin(vectorAngle)*speed, np.cos(vectorAngle)*speed, zV), Vector3(0.0, 0.0, 0.0))
		irpos.set_force_controller_goal(inertia, reciprocaldamping, wrench, twist)
	time.sleep(0.1)	
	wrench = Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
	twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
	irpos.set_force_controller_goal(inertia, reciprocaldamping, wrench, twist)
	print '[OverScript] Main loop done. Attempting to shut down logging component.'
	time.sleep(0.1)	
	irpos.stop_logging()
	time.sleep(0.1)
	irpos.stop_force_controller()
	print '[OverScript] print Cartesian Pose'
	print('[Cartesian pose]')
	print str(irpos.get_cartesian_pose())	
	print 'max zDelta = '+str(maxzDelta)
	time.sleep(1.0)
	#for iter in range(0,719):
	#	setMarker(iter,0.0)
	#	time.sleep(0.005)


	joint_trajectory = [JointTrajectoryPoint([0.0, -1.5707963268, 0.0, 0.0, 4.7123889804 , 0.0], [0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(20.0))]
	irpos.move_along_joint_trajectory(joint_trajectory)
	print "Irp6p: Behavior: TRacking3 - done."

def irp6p_multi_trajectory2():
	irpos = IRPOS("IRpOS", "Irp6p", 6)

	irpos.move_to_motor_position([0.4, -1.5418065817051163, 0.0, 1.57, 1.57, -2.0], 10.0)
	irpos.move_to_motor_position([10.0, 10.0, 0.0, 10.57, 10.57, -20.0], 2.0)

	irpos.move_to_joint_position([0.4, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], 3.0)
	irpos.move_to_joint_position([0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], 3.0)

	rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.18029263241))
	rot2 = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.3, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
	irpos.move_to_cartesian_pose(3.0, Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)))
	irpos.move_to_cartesian_pose(3.0,pm.toMsg(rot))
	irpos.move_to_cartesian_pose(3.0,pm.toMsg(rot2))

	toolParams = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
	irpos.set_tool_geometry_params(toolParams)

	rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 1.4, 3.14), PyKDL.Vector(0.705438961242, -0.1208864692291, 1.181029263241))
	irpos.move_to_cartesian_pose(3.0,Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.675351045979, 0.0892025112399, 0.698321120995, 0.219753244928)))
	irpos.move_to_cartesian_pose(3.0,pm.toMsg(rot))
	irpos.move_to_cartesian_pose(3.0,Pose(Point(0.705438961242, -0.1208864692291, 1.181029263241), Quaternion(0.63691, 0.096783, 0.75634, -0.11369)))

	toolParams = Pose(Point(0.0, 0.0, 0.25), Quaternion(0.0, 0.0, 0.0, 1.0))
	irpos.set_tool_geometry_params(toolParams)

	print "Irp6p 'multi_trajectory2' test compleated"

def irp6p_get_status():
	#irpos = IRPOS("IRpOS", "Irp6p", 6)
	irpos = IRPOS("IRpOS", "Irp6p", 6, "irp6p_manager")
	

	print('[Joint position]')
	print str(irpos.get_joint_position())
	print('[Motor position]')
	print str(irpos.get_motor_position())
	print('[Cartesian pose]')
	print str(irpos.get_cartesian_pose())
	print('[Wrench]')
	print str(irpos.get_force_readings())

	print "Irp6p 'get_status test compleated"

def irp6p_synchro_position():
	irpos = IRPOS("IRpOS", "Irp6p", 6)

	irpos.move_to_synchro_position(10.0)

	print "Irp6p 'synchro_position' test compleated"

# TRACK DEMOS

def irp6otm_multi_trajectory():
		
	print "Irp6otm 'multi_trajectory' test compleated"

def irp6otm_multi_trajectory2():
	irpos = IRPOS("IRpOS", "Irp6ot", 7)

	irpos.move_to_motor_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 10.0)
	irpos.move_to_motor_position([50.0, 10.0, 10.0, 0.0, 10.57, 10.57, -20.0], 2.0)

	irpos.move_to_joint_position([0.0, 0.4, -1.5418065817051163, 0.0, 1.5, 1.57, -2.0], 3.0)
	irpos.move_to_joint_position([0.1, 0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -1.57], 3.0)

	print "Irp6otm 'multi_trajectory2' test compleated"

def irp6otm_get_status():
	irpos = IRPOS("IRpOS", "Irp6ot", 7)

	print('[Joint position]')
	print str(irpos.get_joint_position())
	print('[Motor position]')
	print str(irpos.get_motor_position())
	print('[Cartesian pose]')
	print str(irpos.get_cartesian_pose())
	print('[Wrench]')
	print str(irpos.get_force_readings())

	print "Irp6otm 'get_status' test compleated"

def irp6otm_synchro_position():
	irpos = IRPOS("IRpOS", "Irp6ot", 7)

	irpos.move_to_synchro_position(10.0)

	print "Irp6otm 'synchro_position' test compleated"
	
def test():
	print 'START TEST'

	irpos = IRPOS("IRpOS", "Irp6p", 6)

	irpos.move_to_synchro_position(10.0)
	
	rot = PyKDL.Frame(PyKDL.Rotation.EulerZYZ(0.0, 0.1, 0.1), PyKDL.Vector(0.0, 0.0, 0.0))
	irpos.move_rel_to_cartesian_pose(3.0,pm.toMsg(rot))
	

	print 'END TEST'
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
	q_current_wrist = [currentWristOutputPose.orientation.x, currentWristOutputPose.orientation.y, currentWristOutputPose.orientation.z, currentWristOutputPose.orientation.w]
	qt1 = quaternion_about_axis(0.0, yaxis)
	qt2 = quaternion_about_axis(-1.57079632679, zaxis)
	newRotation = quaternion_multiply(q_current_wrist, qt1)
	newRotation = quaternion_multiply(newRotation, qt2)
	print newRotation
	
	q_sensorFrame = [0.0,0.0,-1.0,0.0]

	q_gravity = [0.0,0.707,0.0,0.707]
	q_gravity_in_wrist = quaternion_multiply(q_gravity, q_current_wrist)

	marker.pose.orientation.x = q_gravity_in_wrist[0]
	marker.pose.orientation.y = q_gravity_in_wrist[1]
	marker.pose.orientation.z = q_gravity_in_wrist[2]
	marker.pose.orientation.w = q_gravity_in_wrist[3]

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
	
	#marker.pose.orientation.x = newRotation[0]
	#marker.pose.orientation.y = newRotation[1]
	#marker.pose.orientation.z = newRotation[2]
	#marker.pose.orientation.w = newRotation[3]

	marker.color.r = 0.0
	marker.color.g = 0.0
	marker.color.b = 1.0
	marker.color.a = 2.0
	rviz_pub.publish(marker)



	print "done"

def Ttest():
	irpos = exirpos("IRpOS", "Irp6p", 6, "irp6p_manager")
	print "Irp6ot: Behavior: arrows."

	current = irpos.get_HandForce2()
	print current

def Tsig():
	irpos = exirpos("IRpOS", "Irp6p", 6, "irp6p_manager")
	print "Sending singling to OptoController"
	#goal = startOptoControllerActionGoal()
	#cartesianGoal = CartesianTrajectoryGoal()
	#goal.goal.OptoGoal = 7
	#print goal
	#print cartesianGoal;
	#client = actionlib.SimpleActionClient('/optoController/commandLine', startOptoControllerAction)
	#client.wait_for_server()
	
	

	#client.send_goal(goal)




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
	elif sys.argv[1]=="TposGrab":
		TposGrab()
	elif sys.argv[1]=="TgetWeightsABC":
		TgetWeightsABC()
	elif sys.argv[1]=="Ttest":
		Ttest()
	elif sys.argv[1]=="Tholder":
		Tholder()
	elif sys.argv[1]=="Tsig":
		Tsig()
