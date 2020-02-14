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

#ifndef USV_GAZEBO_NHAWIGATORA_THRUST_H
#define USV_GAZEBO_NHAWIGATORA_THRUST_H

// C++
#include <algorithm> // min/max
#include <math.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <usv_gazebo_plugins/ThrustConfig.h>
#include <usv_gazebo_plugins/impeller.h>

// Custom Callback Queue
#include <ros/advertise_options.h>
#include <ros/callback_queue.h>

using namespace ignition;

namespace gazebo {
class UsvNhawigatoraThrust : public ModelPlugin {
public:
  UsvNhawigatoraThrust();
  virtual ~UsvNhawigatoraThrust();
  /*! Loads the model in gets dynamic parameters from SDF. */
  virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
  /*! Callback for Gazebo simulation engine */
  virtual void UpdateChild();

private:
  /*!
    Callback for Drive commands
    \param msg std_msgs Float64
  */
  void propellerHandle(const std_msgs::Float64ConstPtr &msg);
  void bowThrusterHandle(const std_msgs::Float64ConstPtr &msg);

  /*! Callback for dynamic reconfigure */
  void dynamicReconfigureHandle(usv_gazebo_plugins::ThrustConfig &config,
                                uint32_t level);

  /*! ROS spin once */
  void spin();

  /*! Convenience function for getting SDF parameters

   */
  double getSdfParamDouble(sdf::ElementPtr sdfPtr,
                           const std::string &param_name, double default_val);

  /// Parameters
  std::string node_namespace_;
  std::string link_name_;

  ros::NodeHandle *rosnode_;

  ros::Subscriber propeller_sub_;
  ros::Subscriber bow_thruster_sub_;

  dynamic_reconfigure::Server<usv_gazebo_plugins::ThrustConfig> server_;

  event::ConnectionPtr update_connection_;
  boost::thread *spinner_thread_;

  /*! Pointer to the Gazebo world, retrieved when the model is loaded */
  physics::WorldPtr world_;
  /*! Pointer to Gazebo parent model, retrieved when the model is loaded */
  physics::ModelPtr model_;
  /*! Pointer to model link in gazebo,
    optionally specified by the bodyName parameter,
    The states are taken from this link and forces applied to this link.*/
  physics::LinkPtr link_;
  /*! Timeout for recieving Drive commands [s]*/
  double cmd_timeout_;
  common::Time prev_update_time_;
  common::Time last_msg_time_;
  double last_msg_proppeller_;
  double last_msg_bowthruster_;

  Impeller bow_thruster_;
  Impeller propeller_;

  /*! Plugin Parameter: Boat width [m] */
  double param_boat_width_;
  /*! Plugin Parameter: Boat length [m] */
  double param_boat_length_;
  /*! Plugin Parameter: Bow thruster X offset from model base */
  double bow_thruster_x_offset_;
  /*! Plugin Parameter: Propeller X offset from model base */
  double propeller_x_offset_;
  /*! Plugin Parameter: Propeller and bow thruster Z offset from model base */
  double thruster_z_offset_;
  /*! Plugin Parameter: Sway factor to apply to torque */
  double sway_factor_;
  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
}; // class UsvNhawigatoraThrust
} // namespace gazebo

#endif // USV_GAZEBO_NHAWIGATORA_THRUST_H
