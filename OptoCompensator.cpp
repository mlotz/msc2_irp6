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

#include <string>

#include <rtt/Component.hpp>

#include "OptoCompensator.h"
#include "eigen_conversions/eigen_msg.h"

OptoCompensator::OptoCompensator(const std::string& name)
    : RTT::TaskContext(name, PreOperational) {
	//Inputs
  this->ports()->addPort("CurrentWristPose", port_current_wrist_pose_);
  this->ports()->addPort("CurrentHandForceInput", port_HandForce_input_); 
  this->ports()->addPort("ToolGravityParam", port_current_tool_gravity_param_);
  this->ports()->addPort("Tool", port_tool_);
	//Outputs
  this->ports()->addPort("OutputCompensatedForce", port_Force_out_);
 
	//Properites
  this->addProperty("sensor_frame", sensor_frame_property_);
  this->addProperty("is_right_turn_frame", is_right_turn_frame_property_);
  this->addProperty("tool_weight", tool_weight_property_);
  this->addProperty("gravity_arm_in_wrist", gravity_arm_in_wrist_property_);
}

OptoCompensator::~OptoCompensator() {
}

bool OptoCompensator::configureHook() {
  tf::poseMsgToKDL(sensor_frame_property_, sensor_frame_kdl_);
  tf::vectorMsgToKDL(gravity_arm_in_wrist_property_, gravity_arm_in_wrist_kdl_);

  return true;
}

bool OptoCompensator::startHook() {
  // read current force ad set as an offset force
  geometry_msgs::Vector3Stamped current_force;
  if (port_HandForce_input_.read(current_force)
      == RTT::NoData) { std::cout<<"[ERROR]:HandForce port is dead."<<std::endl;
    return false;
  }

  //tf::wrenchMsgToKDL(current_force, force_offset_);

  // read current wrist pose
  geometry_msgs::Pose current_wrist_pose;
  if (port_current_wrist_pose_.read(current_wrist_pose) == RTT::NoData) {std::cout<<"[ERROR]:CurrentWristPose port is dead."<<std::endl;
    return false;
  }

  force_control_msgs::ToolGravityParam tg_param;
  if (port_current_tool_gravity_param_.read(tg_param) == RTT::NewData) {
    tool_weight_property_ = tg_param.weight;
    geometry_msgs::Vector3 gravity_arm_in_wrist_property_ = tg_param.mass_center;
    tf::vectorMsgToKDL(gravity_arm_in_wrist_property_, gravity_arm_in_wrist_kdl_);
} else { std::cout<<"[Warning]:No tg_param set - port is dead. Using default tg_param."<<std::endl;}


force_zero = current_force.vector;
force_zero_position = current_wrist_pose;


gravity_force_torque_in_base_ = KDL::Wrench(
      KDL::Vector(0.0, 0.0, -tool_weight_property_),
      KDL::Vector(0.0, 0.0, 0.0));


  return true;
}




void OptoCompensator::updateHook() {
	geometry_msgs::Pose current_wrist_pose;
	port_current_wrist_pose_.read(current_wrist_pose);
	KDL::Frame current_wrist_pose_kdl;
	tf::poseMsgToKDL(current_wrist_pose, current_wrist_pose_kdl);
	
	Eigen::Quaterniond gravityBaseOrientation(0.0, 0.707, 0.0, 0.707); 
	Eigen::Quaterniond base2end(current_wrist_pose.orientation.x,current_wrist_pose.orientation.y,current_wrist_pose.orientation.z,current_wrist_pose.orientation.w);
	Eigen::Quaterniond gravityInEnd = gravityBaseOrientation * base2end ;

	// odczyt sily
	geometry_msgs::Vector3Stamped force_in;
	geometry_msgs::Vector3 current_force,zeroed_force;
	geometry_msgs::Vector3Stamped force_out;
	port_HandForce_input_.read(force_in);
	current_force = force_in.vector;
	//
	
	//
	zeroed_force = force_in.vector;

	zeroed_force.x -= force_zero.x;
	zeroed_force.y -= force_zero.y;
	zeroed_force.z -= force_zero.z;
	
	

	//grav->base
  	//KDL::Wrench gravity_force_torque_in_sensor = (current_wrist_pose_kdl.Inverse()).M * gravity_force_torque_in_base_;
	//KDL::Wrench zeroed_force_wrench(KDL::Vector(zeroed_force.x, zeroed_force.y, zeroed_force.z), KDL::Vector(0.0, 0.0, 0.0));
	//zeroed_force_wrench = sensor_frame_kdl_ * zeroed_force_wrench;
	//KDL::Wrench computed_force = zeroed_force_wrench - gravity_force_torque_in_base_; 
	//computed_force = current_wrist_pose_kdl.M * (-computed_force);
	//geometry_msgs::Wrench force_out_wrench;
	//tf::wrenchKDLToMsg(computed_force, force_out_wrench);
	
	//std::cout<<current_wrist_pose.orientation<<std::endl;
	
	force_out.header.seq = force_in.header.seq;	
	force_out.vector = zeroed_force;

	port_Force_out_.write(force_out);

}

ORO_CREATE_COMPONENT(OptoCompensator)

