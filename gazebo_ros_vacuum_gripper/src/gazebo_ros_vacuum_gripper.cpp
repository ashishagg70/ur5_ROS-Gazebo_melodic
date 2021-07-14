/*
 * Copyright 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboVacuumGripper plugin for manipulating objects in Gazebo
   Author: Kentaro Wada
   Date: 7 Dec 2015
 */

#include <algorithm>
#include <assert.h>

#include <std_msgs/Bool.h>
#include "gazebo_ros_vacuum_gripper/gazebo_ros_vacuum_gripper.h"
#include <gazebo_msgs/SpawnModel.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosVacuumGripper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVacuumGripper::GazeboRosVacuumGripper()
{
  connect_count_ = 0;
  status_ = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVacuumGripper::~GazeboRosVacuumGripper()
{
  update_connection_.reset();

  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();

  delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVacuumGripper::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO_NAMED("vacuum_gripper", "Loading gazebo_ros_vacuum_gripper");

  // Set attached model;
  parent_ = _model;

  // Get the world name.
  world_ = _model->GetWorld();

  // load parameters
  robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("vacuum_gripper", "vacuum_gripper plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  link_ = _model->GetLink(link_name_); //gripper link
  if (!link_)
  {
    std::string found;
    physics::Link_V links = _model->GetLinks();
    for (size_t i = 0; i < links.size(); i++) {
      found += std::string(" ") + links[i]->GetName();
    }
    ROS_FATAL_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper plugin error: link named: %s does not exist", link_name_.c_str());
    ROS_FATAL_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper plugin error: You should check it exists and is not connected with fixed joint");
    ROS_FATAL_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper plugin error: Found links are: %s", found.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("vacuum_gripper", "vacuum_gripper plugin missing <serviceName>, cannot proceed");
    return;
  }
  else
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();
  if (_sdf->HasElement("maxForce")) {
    max_force_ = _sdf->GetElement("maxForce")->Get<double>();
  } else {
    max_force_ = 2;
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("vacuum_gripper", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  rosnode_ = new ros::NodeHandle(robot_namespace_);

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
    topic_name_, 1,
    boost::bind(&GazeboRosVacuumGripper::Connect, this),
    boost::bind(&GazeboRosVacuumGripper::Disconnect, this),
    ros::VoidPtr(), &queue_);
  pub_ = rosnode_->advertise(ao);

  // Custom Callback Queue
  ros::AdvertiseServiceOptions aso1 =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
    "on", boost::bind(&GazeboRosVacuumGripper::OnServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv1_ = rosnode_->advertiseService(aso1);
  ros::AdvertiseServiceOptions aso2 =
    ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
    "off", boost::bind(&GazeboRosVacuumGripper::OffServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv2_ = rosnode_->advertiseService(aso2);

  // Custom Callback Queue
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosVacuumGripper::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosVacuumGripper::UpdateChild, this));

  ROS_INFO_NAMED("vacuum_gripper", "Loaded gazebo_ros_vacuum_gripper");
}

bool GazeboRosVacuumGripper::OnServiceCallback(gazebo_msgs::SpawnModel::Request &req,
                                     gazebo_msgs::SpawnModel::Response &res)
{
  if (status_) {
    ROS_WARN_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: already status is 'on'");
  } else {
    status_ = true;
    pickup_item_name_=req.model_name;
    ROS_INFO_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: status: off -> on");
  }
  return true;
}
bool GazeboRosVacuumGripper::OffServiceCallback(gazebo_msgs::SpawnModel::Request &req,
                                     gazebo_msgs::SpawnModel::Response &res)
{
  if (status_) {
    status_ = false;
    ROS_INFO_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: status: on -> off");
  } else {
    ROS_WARN_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: already status is 'off'");
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVacuumGripper::UpdateChild()
{
  std_msgs::Bool grasping_msg;
  grasping_msg.data = false;
  if (!status_) {
    pub_.publish(grasping_msg);
    return;
  }
  // apply force
  lock_.lock();
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d parent_pose = link_->WorldPose();
  physics::Model_V models = world_->Models();
#else
  ignition::math::Pose3d parent_pose = link_->GetWorldPose().Ign();
  physics::Model_V models = world_->GetModels();
#endif
  //ROS_FATAL_NAMED("vaccum gripper","pickup item name: %s", pickup_item_name_.c_str());
  for (size_t i = 0; i < models.size(); i++) {
    if (models[i]->GetName() == link_->GetName() ||
        models[i]->GetName() == parent_->GetName() || models[i]->GetName().find("red_blocks_") || models[i]->GetName()!=pickup_item_name_)
    {
      continue;
    }
    physics::Link_V links = models[i]->GetLinks();
    for (size_t j = 0; j < links.size(); j++) {
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d link_pose = links[j]->WorldPose();
#else
      ignition::math::Pose3d link_pose = links[j]->GetWorldPose().Ign();
#endif
      ignition::math::Pose3d diff = parent_pose - link_pose;
      double norm = diff.Pos().Length();
      //ROS_FATAL_NAMED("vacuum_gripper", "ashish model_name: %s, link_name: %s", models[i]->GetName().c_str(),link_name_.c_str());
      //ROS_FATAL_NAMED("vacuum_gripper", "ashish maxforce: %lf, norm: %lf, z_gripper: %lf, z_link: %lf", max_force_, norm, parent_pose.Pos().Z(), link_pose.Pos().Z());
      /*if (norm < 0.5) {
#if GAZEBO_MAJOR_VERSION >= 8
        //links[j]->SetLinearVel(link_->WorldLinearVel());
        //links[j]->SetAngularVel(link_->WorldAngularVel());
        links[j]->SetLinearAccel(ignition::math::Vector3d(0, 0, 0));
        links[j]->SetAngularAccel(ignition::math::Vector3d(0, 0, 0));
        links[j]->SetAngularAccel(ignition::math::Vector3d(0, 0, 0));
        links[j]->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
#else
        links[j]->SetLinearVel(link_->GetWorldLinearVel());
        links[j]->SetAngularVel(link_->GetWorldAngularVel());
#endif
        double norm_force = 1 / norm;
        if (norm < 0.05) {
          // apply friction like force
          // TODO(unknown): should apply friction actually
          ignition::math::Vector3d new_pose_box=ignition::math::Vector3d(parent_pose.Pos().X(), parent_pose.Pos().Y(), parent_pose.Pos().Z()+0.01-links[j]->BoundingBox().ZLength()/2.0);
          link_pose.Set(new_pose_box, link_pose.Rot());
          links[j]->SetWorldPose(link_pose);
          ROS_FATAL_NAMED("vacuum_gripper", "ashish maxforce: %lf, norm: %lf, z_gripper: %lf, z_link: %lf", max_force_, norm, parent_pose.Pos().Z(), link_pose.Pos().Z());
        }
         norm_force = max_force_;
        ignition::math::Vector3d force = norm_force * diff.Pos().Normalize();
        links[j]->AddForce(force);
        grasping_msg.data = true;
      }*/
      if(norm<0.5){
        links[j]->SetWorldTwist(link_->WorldLinearVel(), link_->WorldAngularVel(), true);
        links[j]->SetLinearAccel(link_->WorldLinearAccel());
        links[j]->SetAngularAccel(link_->WorldAngularAccel());
        ignition::math::Vector3d new_pose_box=ignition::math::Vector3d(parent_pose.Pos().X(), parent_pose.Pos().Y(), link_pose.Pos().Z());
        ignition::math::Pose3d new_link_pose=ignition::math::Pose3d(new_pose_box, link_pose.Rot());
        links[j]->SetWorldPose(new_link_pose);
        // link_pose.Set(parent_pose.Pos(), link_pose.Rot());
        // links[j]->SetWorldPose(link_pose);
        links[j]->AddForce(10*link_pose.Rot().RotateVector((parent_pose - link_pose).Pos()).Normalize());
      }
      grasping_msg.data = true;
    }
  }
  pub_.publish(grasping_msg);
  lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosVacuumGripper::QueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosVacuumGripper::Connect()
{
  this->connect_count_++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosVacuumGripper::Disconnect()
{
  this->connect_count_--;
}

}  // namespace gazebo