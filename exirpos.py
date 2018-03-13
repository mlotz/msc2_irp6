#!/usr/bin/env python

from irpos import *

class exirpos(IRPOS):
	
	seamless_relative_movement = 0
	OKGREEN = '\033[92m'
	ENDC = '\033[0m'
	REDFAIL = '\033[91m'
	PoseInt_ON = 0
	
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
		
#MAIN

if __name__ == '__main__':
	print 'main'
