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

OptoController::OptoController(const std::string& name)
    : RTT::TaskContext(name, PreOperational),
      step_duration_(0.002) {

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

  as_.addPorts(this->provides());
 
  this->addEventPort(command_port_, boost::bind(&OptoController::commandCB, this));
  as_.registerGoalCallback(boost::bind(&OptoController::executeCB, this, _1));
  //as_.registerCancelCallback(boost::bind(&InternalSpaceSplineTrajectoryAction::cancelCB, this, _1));
}

OptoController::~OptoController() {
}

bool OptoController::configureHook() {
  return true;
}

bool OptoController::startHook() {
	//bool is_synchronised = true;
	tfgJointInput_.read(current_tfgJoint);
	port_generator_active_.write(true);
	as_.start();

	goal_active_ = false;
	enable_ = true;	
	
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
	//port_HandForce1_in_.read(H1);
	//port_HandForce2_in_.read(H2);
	
	
	if(current_tfgJoint[0] > 0.065){
		current_tfgJoint[0] = current_tfgJoint[0] - 0.000001;	
	}
	//std::cout<<current_tfgJoint[0]<<std::endl;
	tfgJointOutput_.write(current_tfgJoint);


  //port_output_end_effector_pose_.write(port_current_end_effector_pose_);
}

double OptoController::fcl(const double & rdam, const double & inertia,
                            const double & fm, const double & fd,
                            const double & dvel, const double & pvel) {
  return ((rdam * (fd - fm) + dvel) * step_duration_ + rdam * inertia * pvel)
      / (step_duration_ + rdam * inertia);
}

void OptoController::executeCB(GoalHandle gh) {
	std::cout<<"i hear you goal"<<std::endl;
	return;
}

void OptoController::commandCB() {
	std::cout<<"i hear you command"<<std::endl;
	return;
}

ORO_CREATE_COMPONENT(OptoController)
