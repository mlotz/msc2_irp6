/*
 * Copyright (c) 2014-2015, Robot Control and Pattern Recognition Group, Warsaw University of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robot Control and Pattern Recognition Group,
 *       Warsaw University of Technology nor the names of its contributors may
 *       be used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <rtt/Component.hpp>
#include <string>

#include "OptoController.h"
#include "eigen_conversions/eigen_msg.h"
#include "kdl_conversions/kdl_msg.h"

OptoController::OptoController(const std::string& name) : RTT::TaskContext(name, PreOperational), step_duration_(0.002) {

	this->ports()->addPort("CurrentEndEffectorPose",
                         port_current_end_effector_pose_);
	this->ports()->addPort("OutputEndEffectorPose",
                         port_output_end_effector_pose_);

	this->ports()->addPort("port_HandForce1_in", port_HandForce1_in_);
	this->ports()->addPort("port_HandForce2_in", port_HandForce2_in_);

	this->ports()->addPort("CurrentFclParam", port_current_fcl_param_);
	this->ports()->addPort("GeneratorActiveOut", port_generator_active_);
	this->ports()->addPort("IsSynchronisedIn", port_is_synchronised_);
  
	this->ports()->addPort("tfgJointInput", tfgJointInput_);
	this->ports()->addPort("tfgJointOutput", tfgJointOutput_);

	tfgDelta = -0.000001;
	cartesianYDelta = -0.004;

//Actionlib
  //as_.addPorts(this->provides());
  //this->addEventPort(command_port_, boost::bind(&OptoController::commandCB, this));
  //as_.registerGoalCallback(boost::bind(&OptoController::goalCB, this, _1));
  //as_.registerCancelCallback(boost::bind(&OptoController::cancelCB, this, _1));
}

OptoController::~OptoController() {
}

bool OptoController::configureHook() {
	return true;
}

bool OptoController::startHook() {
	//bool is_synchronised = true;
	tfgJointInput_.read(current_tfgJoint);
	//port_current_end_effector_pose_.read(cartesianPoseVar);
	port_generator_active_.write(true);

	//
	geometry_msgs::Pose cl_ef_pose;
	if (port_current_end_effector_pose_.read(cl_ef_pose) == RTT::NoData) {
		return false;
	}
	tf::poseMsgToKDL(cl_ef_pose, cl_ef_pose_kdl_);

	//actionLib
	//as_.start();
	//goal_active_ = false;
	//enable_ = true;	
	
  	return true;
}

void OptoController::stopHook() {
	port_generator_active_.write(false);
	//as_.stop();
}

void OptoController::updateHook() {
	port_generator_active_.write(true);
	// current wrench determination
	geometry_msgs::Vector3Stamped H1,H2;



	



	
	
	//--------------------------TGF section----------


	//port_current_fcl_param_.read(fcl_param);
	port_HandForce1_in_.read(H1);
	port_HandForce2_in_.read(H2);
	//Goal g = activeGoal_.getGoal();
	
	//msc_mlotz_pkg::startOptoControllerResult res;
	//res.OptoResult = 0;
	//activeGoal_.setSucceeded(res, "");
	if(tfgDelta<0.0){
		if(current_tfgJoint[0] > 0.062){
			//std::cout<<H1.vector.z<<"||"<<H2.vector.z<<std::endl;
			//asd
			if(H1.vector.z<0.8 || H2.vector.z<0.8){
			current_tfgJoint[0] = current_tfgJoint[0] + tfgDelta;
			}	
		}
		else
		{
		tfgDelta = -tfgDelta;
		}
	} else
	{
		if(current_tfgJoint[0] < 0.075){
			current_tfgJoint[0] = current_tfgJoint[0] + 4*tfgDelta;	
		}
		else
		{
		tfgDelta = -tfgDelta;
		}
	}

	
	
	// ------------ Cartesian section ---------
	cartesianYDelta = 0.01*H1.vector.z - 0.003 ;
	if (cartesianYDelta > 0.004) {cartesianYDelta = 0.004;}
	if (cartesianYDelta < -0.004) {cartesianYDelta = -0.004;}
	if(H1.vector.z>0.8 && H2.vector.z>0.8){cartesianYDelta = 0;}
	
	


	//DEBUG_Data
	//Waiting for actionlib
	force_control_msgs::ForceControl fcl_param;
	fcl_param.reciprocaldamping.translation.x=0.0025;
	fcl_param.reciprocaldamping.translation.y=0.0025;
	fcl_param.reciprocaldamping.translation.z=0.0025;
	fcl_param.reciprocaldamping.rotation.x=0.05;
	fcl_param.reciprocaldamping.rotation.y=0.05;
	fcl_param.reciprocaldamping.rotation.z=0.05;
	fcl_param.inertia.translation.x=20.0;
	fcl_param.inertia.translation.y=20.0;
	fcl_param.inertia.translation.z=20.0;
	fcl_param.inertia.rotation.x=0.5;
	fcl_param.inertia.rotation.y=0.5;
	fcl_param.inertia.rotation.z=0.5;
	fcl_param.wrench.force.x=0.0;
	fcl_param.wrench.force.y=0.0;
	fcl_param.wrench.force.z=0.0;
	fcl_param.wrench.torque.x=0.0;
	fcl_param.wrench.torque.y=0.0;
	fcl_param.wrench.torque.z=0.0;
	fcl_param.twist.linear.x=0.0;
	fcl_param.twist.linear.y=cartesianYDelta;
	fcl_param.twist.linear.z=0.0;
	fcl_param.twist.angular.x=0.0;
	fcl_param.twist.angular.y=0.0;
	fcl_param.twist.angular.z=0.0;
	KDL::Wrench input_force;
	tf::wrenchMsgToKDL(fcl_param.wrench, input_force);
	//input_force.force.x=0.0;
	//input_force.force.y=0.0;
	//input_force.force.z=0.0;
	//input_force.torque.x=0.0;
	//input_force.torque.y=0.0;
	//input_force.torque.z=0.0;
	//std::cout<<fcl_param<<std::endl;
	
	//debug end
	

	//-----------------cartesian section---------------
	KDL::Twist target_vel;
	
	target_vel.vel[0] = fcl(fcl_param.reciprocaldamping.translation.x,
		                  fcl_param.inertia.translation.x,
		                  input_force.force.x(), fcl_param.wrench.force.x,
		                  fcl_param.twist.linear.x, p_vel_.vel[0]);

	target_vel.vel[1] = fcl(fcl_param.reciprocaldamping.translation.y,
		                  fcl_param.inertia.translation.y,
		                  input_force.force.y(), fcl_param.wrench.force.y,
		                  fcl_param.twist.linear.y, p_vel_.vel[1]);

	target_vel.vel[2] = fcl(fcl_param.reciprocaldamping.translation.z,
		                  fcl_param.inertia.translation.z,
		                  input_force.force.z(), fcl_param.wrench.force.z,
		                  fcl_param.twist.linear.z, p_vel_.vel[2]);

	target_vel.rot[0] = fcl(fcl_param.reciprocaldamping.rotation.x,
		                  fcl_param.inertia.rotation.x, input_force.torque.x(),
		                  fcl_param.wrench.torque.x, fcl_param.twist.angular.x,
		                  p_vel_.rot[0]);

	target_vel.rot[1] = fcl(fcl_param.reciprocaldamping.rotation.y,
		                  fcl_param.inertia.rotation.y, input_force.torque.y(),
		                  fcl_param.wrench.torque.y, fcl_param.twist.angular.y,
		                  p_vel_.rot[1]);

	target_vel.rot[2] = fcl(fcl_param.reciprocaldamping.rotation.z,
		                  fcl_param.inertia.rotation.z, input_force.torque.z(),
		                  fcl_param.wrench.torque.z, fcl_param.twist.angular.z,
		                  p_vel_.rot[2]);

	p_vel_ = target_vel;

	target_vel = cl_ef_pose_kdl_.M * target_vel;

	cl_ef_pose_kdl_ = KDL::addDelta(cl_ef_pose_kdl_, target_vel, step_duration_);

	geometry_msgs::Pose cl_ef_pose;

	tf::poseKDLToMsg(cl_ef_pose_kdl_, cl_ef_pose);

	port_output_end_effector_pose_.write(cl_ef_pose);
	


	
	//feedback_.OptoState = 7;
	//activeGoal_.publishFeedback(feedback_);
	tfgJointOutput_.write(current_tfgJoint);
	


 
}

double OptoController::fcl(const double & rdam, const double & inertia, const double & fm, const double & fd, const double & dvel, const double & pvel) {
	return ((rdam * (fd - fm) + dvel) * step_duration_ + rdam * inertia * pvel) / (step_duration_ + rdam * inertia);
}
/*
void OptoController::goalCB(GoalHandle gh) {
	goal_active_ = true;
	Goal g = gh.getGoal();
	activeGoal_ = gh;
	std::cout<<"i hear your goal"<<std::endl;
	return;
}

void OptoController::commandCB() {
	std::cout<<"i hear your command"<<std::endl;
	return;
}

void OptoController::cancelCB(GoalHandle gh) {
  std::cout<<"OptoController cancel"<<std::endl;
  goal_active_ = false;
}
*/
ORO_CREATE_COMPONENT(OptoController)
