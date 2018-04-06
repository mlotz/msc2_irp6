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
  this->ports()->addPort("OutputSensorPose", port_SensorPose_out_);
  this->ports()->addPort("OutputZeroedForce", port_zeroed_Force_out_);
 
	//Properites
  this->addProperty("sensor_frame", sensor_frame_property_);
  this->addProperty("is_right_turn_frame", is_right_turn_frame_property_);
  this->addProperty("tool_weight", tool_weight_property_);
  this->addProperty("gravity_arm_in_wrist", gravity_arm_in_wrist_property_);
  this->addProperty("weightInBpos", weightInBpos_);
  this->addProperty("wrench2sensor_R_param", wrench2sensor_R_param_);
  this->addProperty("zeroing_offset", zeroing_offset_);
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
KDL::Frame current_wrist_pose_kdl;
tf::poseMsgToKDL(current_wrist_pose, current_wrist_pose_kdl);



zeroWrench_KDL = KDL::Wrench(KDL::Vector(current_force.vector.x, current_force.vector.y, current_force.vector.z), KDL::Vector(0.0,0.0,0.0));



//gravity_force_torque_in_base_ = KDL::Wrench(
//      KDL::Vector(0.0, 0.0, -tool_weight_property_),
//      KDL::Vector(0.0, 0.0, 0.0));
gravity_force_torque_in_base_ = KDL::Wrench(
      KDL::Vector(-weightInBpos_.x, -weightInBpos_.y, -weightInBpos_.z),
      KDL::Vector(0.0, 0.0, 0.0));

//zeroWrench_KDL = ZeroWrench_KDL - gravity_force_torque_in_base_;

tool_mass_center_translation_ = KDL::Frame(KDL::Rotation(),
                                             gravity_arm_in_wrist_kdl_);


  return true;
}




void OptoCompensator::updateHook() {
	geometry_msgs::Pose current_wrist_pose;
	port_current_wrist_pose_.read(current_wrist_pose);
	KDL::Frame current_wrist_pose_kdl;
	tf::poseMsgToKDL(current_wrist_pose, current_wrist_pose_kdl);

	
	
	//Eigen::Quaterniond gravityBaseOrientation(0.0, 0.707, 0.0, 0.707); 
	//Eigen::Quaterniond base2end(current_wrist_pose.orientation.x,current_wrist_pose.orientation.y,current_wrist_pose.orientation.z,current_wrist_pose.orientation.w);
	//Eigen::Quaterniond gravityInEnd = gravityBaseOrientation * base2end ;

	// odczyt sily
	geometry_msgs::Vector3Stamped force_in;
	geometry_msgs::Vector3 current_force,zeroed_force,compensated_force;
	geometry_msgs::Vector3Stamped force_out, zeroed_force_out;
	port_HandForce_input_.read(force_in);
	current_force = force_in.vector;
	KDL::Wrench current_force_KDL = KDL::Wrench(
      KDL::Vector(current_force.x, current_force.y, current_force.z),
      KDL::Vector(0.0, 0.0, 0.0)); 
	KDL::Wrench zeroed_force_KDL;
	KDL::Frame force_zero_position_kdl;
	tf::poseMsgToKDL(force_zero_position, force_zero_position_kdl);
	//zerowanie czujnika
	//zeroed_force = force_in.vector;

	//zeroed_force.x -= force_zero.x;
	//zeroed_force.y -= force_zero.y;
	//zeroed_force.z -= force_zero.z;
	zeroed_force_KDL = current_force_KDL;// - zeroWrench_KDL;
	geometry_msgs::Wrench zeroed_force_geom;
	tf::wrenchKDLToMsg(zeroed_force_KDL, zeroed_force_geom);
	zeroed_force.x = zeroed_force_geom.force.x;
	zeroed_force.y = zeroed_force_geom.force.y;
	zeroed_force.z = zeroed_force_geom.force.z;
	
	//Rotacja R w ukladzie (0) przylozona do ukladu nadgarstka
	KDL::Rotation R;
	//R.DoRotX(M_PI/2);
	//R.DoRotZ(M_PI/2);
	//R.DoRotX(M_PI);
	R = R.Quaternion((double)wrench2sensor_R_param_.x,(double)wrench2sensor_R_param_.y,(double)wrench2sensor_R_param_.z,(double)wrench2sensor_R_param_.w);
	KDL::Frame T_pose_kdl = current_wrist_pose_kdl ;
	//Przejscie z ukladu nadgarstka do czujnika

	//double ox,oy,oz,ow;
	//R.GetQuaternion(ox,oy,oz,ow);
	//std::cout<<"x:"<<ox<<"y:"<<oy<<"z:"<<oz<<"w:"<<ow<<std::endl;
	//std::cout<<wrench2sensor_R_param_<<std::endl;
	T_pose_kdl.M = current_wrist_pose_kdl.M*R;
	geometry_msgs::Pose T_sensorPose_out;
	tf::poseKDLToMsg(T_pose_kdl, T_sensorPose_out);
	//z czujnika do (0)
	KDL::Frame T_poseZero_kdl = current_wrist_pose_kdl;
	T_poseZero_kdl.M = (T_pose_kdl.Inverse()).M*T_pose_kdl.M;
	//
	KDL::Wrench zeroed_force_KDL_in0 = (T_pose_kdl.Inverse()).M * zeroed_force_KDL;
	//
	//w tym miejscudzialania w ukladzie zerowym!
	//zeroed_force_KDL_in0 = zeroed_force_KDL_in0 - gravity_force_torque_in_base_;
	//zeroed_force_KDL_in0 = zeroed_force_KDL - zeroed_force_KDL_in0 - gravity_force_torque_in_base_;
	//KDL::Wrench compForce = -(zeroed_force_KDL_in0 - gravity_force_torque_in_base_ - zeroWrench_KDL);
	//KDL::Wrench compForce = zeroed_force_KDL - zeroWrench_KDL - gravity_force_torque_in_base_;
	//KDL::Wrench compForce = zeroed_force_KDL_in0;

	//v1
	//KDL::Wrench compForce = zeroed_force_KDL_in0 + gravity_force_torque_in_base_ - zeroWrench_KDL;

	KDL::Wrench compForce = (T_pose_kdl.Inverse()).M * zeroed_force_KDL  + gravity_force_torque_in_base_ + (T_pose_kdl.Inverse()).M *gravity_force_torque_in_base_;

	//Best	 
	KDL::Wrench compForceV2 = zeroed_force_KDL + T_pose_kdl.M * gravity_force_torque_in_base_ + gravity_force_torque_in_base_;


	// V3
	compForce = (T_pose_kdl.Inverse()).M * zeroed_force_KDL  + gravity_force_torque_in_base_ + (T_pose_kdl.Inverse()).M *gravity_force_torque_in_base_;
	
	//compForceV2 = (T_pose_kdl.Inverse()).M * (compForceV2);

	//
	//powrot z (0) do ukladu czujnika
	T_poseZero_kdl.M = (T_pose_kdl).M*T_poseZero_kdl.M ;
	geometry_msgs::Pose T_poseZero_out;
	tf::poseKDLToMsg(T_poseZero_kdl, T_poseZero_out);
	//port_SensorPose_out_.write(T_poseZero_out);
	port_SensorPose_out_.write(T_sensorPose_out);
	//
	compForce = (T_pose_kdl).M * (compForce);
	geometry_msgs::Wrench forceHolder;
  	tf::wrenchKDLToMsg(compForce, forceHolder);
	//compensated_force.x = forceHolder.force.x -weightInBpos_.x;
	//compensated_force.y = forceHolder.force.y -weightInBpos_.y;
	//compensated_force.z = forceHolder.force.z -weightInBpos_.z; //Dafuq?
	compensated_force.x = forceHolder.force.x +zeroing_offset_.x;
	compensated_force.y = forceHolder.force.y +zeroing_offset_.y;
	compensated_force.z = forceHolder.force.z +zeroing_offset_.z;
	

	
	//Wyjsciowy wektor sily
	force_out.header.seq = force_in.header.seq;	
	force_out.vector = compensated_force;

	port_Force_out_.write(force_out);
	
	//porownawczy wyzerowany wektor sily
	zeroed_force_out.header.seq = force_in.header.seq;
	zeroed_force_out.vector = zeroed_force;
	
	port_zeroed_Force_out_.write(zeroed_force_out);
}

ORO_CREATE_COMPONENT(OptoCompensator)

