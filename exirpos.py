#!/usr/bin/env python

from irpos import *

class exirpos(IRPOS):
	
	seamless_relative_movement = 0
	OKGREEN = '\033[92m'
	ENDC = '\033[0m'
	REDFAIL = '\033[91m'
	PoseInt_ON = 0
	WristOutputPose_cartesian_position_subscriber = None
	last_WristOutputPose_cartesian_position = None	
	WristOutputPose_lock = threading.Lock()

	SensorPose_subscriber = None
	last_SensorPose = None	
	SensorPose_lock = threading.Lock()

	HandForce1_subscriber = None
	last_HandForce1 = None	
	HandForce1_lock = threading.Lock()
	HandForce2_subscriber = None
	last_HandForce2 = None	
	HandForce2_lock = threading.Lock()

	def WristOutputPose_cartesian_position_callback(self, data):
		self.WristOutputPose_lock.acquire()
		self.last_WristOutputPose_cartesian_position = data
		self.WristOutputPose_lock.release()
	def HandForce1_callback(self, data):
		self.HandForce1_lock.acquire()
		self.last_HandForce1 = data
		self.HandForce1_lock.release()
	def HandForce2_callback(self, data):
		self.HandForce2_lock.acquire()
		self.last_HandForce2 = data
		self.HandForce2_lock.release()	
	def SensorPose_callback(self, data):
		self.SensorPose_lock.acquire()
		self.last_SensorPose = data
		self.SensorPose_lock.release()	

	def __init__(self, nodeName, robotName, robotJointNumbers, scheme_name):
		self.WristOutputPose_cartesian_position_subscriber = rospy.Subscriber('/irp6p_arm/WristOutputPose', Pose, self.WristOutputPose_cartesian_position_callback)
		self.HandForce1_subscriber = rospy.Subscriber('/optoforce1/force0_scaled', Vector3Stamped, self.HandForce1_callback)
		self.HandForce2_subscriber = rospy.Subscriber('/optoforce2/force0_scaled', Vector3Stamped, self.HandForce2_callback)
		self.SensorPose_subscriber = rospy.Subscriber('/optoCompensator1/sensor_pose', Pose, self.SensorPose_callback)
		#print self.WristOutputPose_cartesian_position_subscriber
		print(self.OKGREEN+'[EXIRPOS] Class initiated.'+self.ENDC)
		IRPOS.__init__(self, nodeName, robotName, robotJointNumbers, scheme_name)
		
		


	def seamless_move_rel_to_cartesian_pose(self, time_from_start, rel_pose):

		#WARNING:
		#Use only for short,slow movements.
		#wait minimum 100ms before and after calling this function.
		
		if (not(self.PoseInt_ON)):
			print(self.OKGREEN+'[EXIRPOS]'+self.REDFAIL+'!Error: '+self.robot_name+'mPoseInt'+' component is OFF!'+self.ENDC)
			return
			

		actual_pose = self.get_cartesian_pose()

		# Transform poses to frames.
		actualFrame = pm.fromMsg(actual_pose)
		
		relativeFrame = pm.fromMsg(rel_pose)
		
		desiredFrame = actualFrame * relativeFrame
		
		pose = pm.toMsg(desiredFrame)

		cartesianGoal = CartesianTrajectoryGoal()
		cartesianGoal.trajectory.points.append(CartesianTrajectoryPoint(rospy.Duration(time_from_start), pose, Twist()))
		cartesianGoal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.002)
		
		self.pose_client.send_goal(cartesianGoal)
		self.pose_client.wait_for_result()

		result = self.pose_client.get_result()
		code = self.cartesian_error_code_to_string(result.error_code)
		#print self.BCOLOR+"[IRPOS] Result: "+str(code)+self.ENDC

		

	def on_pose_int(self):
		self.conmanSwitch([self.robot_name+'mPoseInt'], [], True)
		print(self.OKGREEN+'[EXIRPOS] Started '+self.robot_name+'mPoseInt'+' component. Ready for seamless relative movements.'+self.ENDC)
		self.PoseInt_ON = 1
	def off_pose_int(self):
		self.conmanSwitch([], [self.robot_name+'mPoseInt'], True)
		print(self.OKGREEN+'[EXIRPOS] STOPPED '+self.robot_name+'mPoseInt'+' component.Seamless relative movements disabled.'+self.ENDC)
		self.PoseInt_ON= 0
	def EX_print_wrench(self):
		self.conmanSwitch([self.robot_name+'mForceTransformation'], [], True)
		time.sleep(0.05)
		print('[Cartesian pose]')
		print str(self.get_force_readings())
		self.conmanSwitch([], [self.robot_name+'mForceTransformation'], True)

	def start_logging(self):
		self.conmanSwitch(['repoUni'], [], True)
		print(self.OKGREEN+'[EXIRPOS] Started logging'+' components.'+self.ENDC)
	def stop_logging(self):
		self.conmanSwitch([], ['repoUni'], True)
		print(self.OKGREEN+'[EXIRPOS] Stopped logging'+' components.'+self.ENDC)
		#self.PoseInt_ON= 0

	def get_WristOutputPose_cartesian_pose(self):
		self.WristOutputPose_lock.acquire()
		ret = self.last_WristOutputPose_cartesian_position
		#print ret
		self.WristOutputPose_lock.release()		
		return ret
	def get_HandForce1(self):
		self.HandForce1_lock.acquire()
		ret = self.last_HandForce1
		self.HandForce1_lock.release()		
		return ret
	def get_HandForce2(self):
		self.HandForce2_lock.acquire()
		ret = self.last_HandForce2
		self.HandForce2_lock.release()		
		return ret
	def get_SensorPose(self):
		self.SensorPose_lock.acquire()
		ret = self.last_SensorPose
		self.SensorPose_lock.release()		
		return ret
		
#MAIN

if __name__ == '__main__':
	print 'main'
