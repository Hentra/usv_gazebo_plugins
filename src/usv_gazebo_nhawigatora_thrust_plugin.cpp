/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the usv_gazebo_dynamics_plugin package, known as this
Package.

This Package free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This Package s distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this package.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <boost/thread.hpp>
#include <ros/time.h>

#include <usv_gazebo_plugins/usv_gazebo_nhawigatora_thrust_plugin.h>

using namespace gazebo;

UsvNhawigatoraThrust::UsvNhawigatoraThrust()
    : server_(ros::NodeHandle("~usv_gazebo_nhawigatora_thrust")),
      bow_thruster_(100, 100), propeller_(100, 100) {}

UsvNhawigatoraThrust::~UsvNhawigatoraThrust() {
  rosnode_->shutdown();
  spinner_thread_->join();
  delete rosnode_;
  delete spinner_thread_;
}

void UsvNhawigatoraThrust::Load(physics::ModelPtr _parent,
                                sdf::ElementPtr _sdf) {
  ROS_INFO("Loading usv_gazebo_nhawigatora_thrust_plugin");
  model_ = _parent;
  world_ = model_->GetWorld();

  // Retrieve model paramters from SDF
  // Set default values
  node_namespace_ = "usv_gazebo_nhawigatora_thrust";
  cmd_timeout_ = 1.0; // how long to allow no input on cmd_drive
  param_boat_width_ = 1.0;
  param_boat_length_ = 1.35;
  thruster_z_offset_ = -0.01;
  propeller_x_offset_ = 0.0;
  bow_thruster_x_offset_ = 0.0;
  sway_factor_ = 0.0;

  bow_thruster_ = Impeller(100.0, -100.0);
  propeller_ = Impeller(100.0, -100.0);

  usv_gazebo_plugins::ThrustConfig thrustConfig;

  //  Enumerating model
  ROS_INFO_STREAM("Enumerating Model...");
  ROS_INFO_STREAM("Model name = " << model_->GetName());
  physics::Link_V links = model_->GetLinks();
  for (unsigned int ii = 0; ii < links.size(); ii++) {
    ROS_INFO_STREAM("Link: " << links[ii]->GetName());
  }

  // Get parameters from SDF
  if (_sdf->HasElement("robotNamespace")) {
    node_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  if (!_sdf->HasElement("bodyName") ||
      !_sdf->GetElement("bodyName")->GetValue()) {
    link_ = model_->GetLink();
    link_name_ = link_->GetName();
    ROS_INFO_STREAM("Did not find SDF parameter bodyName");
  } else {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link_ = model_->GetLink(link_name_);

    ROS_INFO_STREAM("Found SDF parameter bodyName as <" << link_name_ << ">");
  }
  if (!link_) {
    ROS_FATAL("usv_gazebo_thrust_plugin error: bodyName: %s does not exist\n",
              link_name_.c_str());
    return;
  } else {
    ROS_INFO_STREAM("USV Model Link Name = " << link_name_);
  }

  cmd_timeout_ = getSdfParamDouble(_sdf, "cmdTimeout", cmd_timeout_);

  if (_sdf->HasElement("mappingType") &&
      _sdf->GetElement("mappingType")->GetValue()) {
    thrustConfig.mapping_type = _sdf->GetElement("mappingType")->Get<int>();
    ROS_INFO_STREAM("Parameter found - setting <mappingType> to <"
                    << thrustConfig.mapping_type << ">.");
  } else {
    ROS_INFO_STREAM(
        "Parameter <mappingType> not found: Using default value of <"
        << propeller_.getThrustMapping() << ">.");
  }
  thrustConfig.bow_thruster_fwd = getSdfParamDouble(
      _sdf, "bowThrusterMaxForceFwd", bow_thruster_.getMaxForceFwd());
  thrustConfig.bow_thruster_rev = getSdfParamDouble(
      _sdf, "bowThrusterMaxForceRev", bow_thruster_.getMaxForceRev());
  thrustConfig.propeller_fwd = getSdfParamDouble(_sdf, "propellerMaxForceFwd",
                                                 propeller_.getMaxForceFwd());
  thrustConfig.propeller_rev = getSdfParamDouble(_sdf, "propellerMaxForceRev",
                                                 propeller_.getMaxForceRev());
  thrustConfig.sway_factor = getSdfParamDouble(_sdf, "swayFactor", sway_factor_);
  param_boat_width_ = getSdfParamDouble(_sdf, "boatWidth", param_boat_width_);
  param_boat_length_ =
      getSdfParamDouble(_sdf, "boatLength", param_boat_length_);
  thruster_z_offset_ =
      getSdfParamDouble(_sdf, "thrusterOffsetZ", thruster_z_offset_);
  bow_thruster_x_offset_ =
      getSdfParamDouble(_sdf, "bowThrusterXOffset", bow_thruster_x_offset_);
  propeller_x_offset_ =
      getSdfParamDouble(_sdf, "propellerXOffset", propeller_x_offset_);

  // initialize time and odometry position
  prev_update_time_ = last_msg_time_ = this->world_->SimTime();

  // Initialize the ROS node and subscribe to cmd_drive
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "usv_nhawigatora_thrust_gazebo",
            ros::init_options::NoSigintHandler |
                ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(node_namespace_);

  server_.setCallback(boost::bind(
      &UsvNhawigatoraThrust::dynamicReconfigureHandle, this, _1, _2));
  server_.updateConfig(thrustConfig);

  propeller_sub_ =
      rosnode_->subscribe("/motor_controller_node/set_propeller", 1,
                          &UsvNhawigatoraThrust::propellerHandle, this);
  bow_thruster_sub_ =
      rosnode_->subscribe("/motor_controller_node/set_thruster", 1,
                          &UsvNhawigatoraThrust::bowThrusterHandle, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->spinner_thread_ =
      new boost::thread(boost::bind(&UsvNhawigatoraThrust::spin, this));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&UsvNhawigatoraThrust::UpdateChild, this));
}

double UsvNhawigatoraThrust::getSdfParamDouble(sdf::ElementPtr sdfPtr,
                                               const std::string &param_name,
                                               double default_val) {
  double val = default_val;
  if (sdfPtr->HasElement(param_name) &&
      sdfPtr->GetElement(param_name)->GetValue()) {
    val = sdfPtr->GetElement(param_name)->Get<double>();
    ROS_INFO_STREAM("Parameter found - setting <" << param_name << "> to <"
                                                  << val << ">.");

  } else {
    ROS_INFO_STREAM("Parameter <" << param_name
                                  << "> not found: Using default value of <"
                                  << val << ">.");
  }
  return val;
}

void UsvNhawigatoraThrust::dynamicReconfigureHandle(
    usv_gazebo_plugins::ThrustConfig &config, uint32_t level) {
  ROS_INFO("Got reconfigure request: btf=%f;btr=%f;pf=%f;pr=%f",
           config.bow_thruster_fwd, config.bow_thruster_rev,
           config.propeller_fwd, config.propeller_rev);
  propeller_.setMaxForceFwd(config.propeller_fwd);
  propeller_.setMaxForceRev(config.propeller_rev);
  bow_thruster_.setMaxForceFwd(config.bow_thruster_fwd);
  bow_thruster_.setMaxForceRev(config.bow_thruster_rev);
  ThrustMapping thrustMapping = static_cast<ThrustMapping>(config.mapping_type);
  bow_thruster_.setThrustMapping(thrustMapping);
  propeller_.setThrustMapping(thrustMapping);
  sway_factor_ = config.sway_factor;
}

void UsvNhawigatoraThrust::UpdateChild() {
  common::Time time_now = this->world_->SimTime();
  prev_update_time_ = time_now;

  // Enforce command timeout
  double dcmd = (time_now - last_msg_time_).Double();
  if ((dcmd > cmd_timeout_) && (cmd_timeout_ > 0.0)) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Command timeout!");
    last_msg_bowthruster_ = 0.0;
    last_msg_proppeller_ = 0.0;
  }
  // Scale commands to thrust and torque forces
  ROS_DEBUG_STREAM_THROTTLE(
      1.0, "Last cmd: Propeller:" << last_msg_proppeller_ << " Bow thruster: "
                                  << last_msg_bowthruster_);
  double thrust_propeller = 0.0;
  double thrust_bowthruster = 0.0;

  thrust_propeller = propeller_.scaleThrust(last_msg_proppeller_);
  thrust_bowthruster = bow_thruster_.scaleThrust(last_msg_bowthruster_);

  double thrust = thrust_propeller;
  double torque = thrust_bowthruster;
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Thrust: propeller:" << thrust_propeller
                                                      << " bow_thruster: "
                                                      << thrust_bowthruster);

  math::Vector3d propeller_relpos(-1.0 * param_boat_length_ / 2.0, 0.0,
                        thruster_z_offset_); // relative pos of thrusters
  math::Vector3d propeller_force(thrust, 0, 0);

  math::Vector3d thruster_relpos(param_boat_length_ / 1.8, 0.0,
                        thruster_z_offset_); // relative pos of thrusters
  math::Vector3d thruster_force(0, torque, 0);

  math::Pose3d pose = link_->WorldPose();
  propeller_force = pose.Rot().RotateVector(propeller_force);
  thruster_force = pose.Rot().RotateVector(thruster_force);

  link_->AddForceAtRelativePosition(thruster_force, thruster_relpos);
  link_->AddForceAtRelativePosition(propeller_force, propeller_relpos);
}

void UsvNhawigatoraThrust::propellerHandle(
    const std_msgs::Float64ConstPtr &msg) {
  last_msg_time_ = this->world_->SimTime();
  last_msg_proppeller_ = msg->data;
}

void UsvNhawigatoraThrust::bowThrusterHandle(
    const std_msgs::Float64ConstPtr &msg) {
  last_msg_time_ = this->world_->SimTime();
  last_msg_bowthruster_ = msg->data;
}

void UsvNhawigatoraThrust::spin() {

  while (ros::ok())
    ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(UsvNhawigatoraThrust);
