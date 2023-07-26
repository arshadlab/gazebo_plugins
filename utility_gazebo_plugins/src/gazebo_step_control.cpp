// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <utility_ros2_interface/srv/step_control.hpp>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

const size_t SHARED_MEMORY_SIZE = 1 * 1024 * 1024;
//#define STORAGE_ID "/tmp/cosim/physics2"
#define STORAGE_ID "physics"
class GazeboStepControl : public gazebo::WorldPlugin
{
public:
  /// Constructor
  GazeboStepControl();
  ~GazeboStepControl();
  // Documentation inherited
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

  /// Callback when a world is created.
  /// \param[in] _world_name The world's name
  void OnWorldCreated(const std::string & _world_name);

  /// Publish simulation time.
  /// \param[in] _info World update information.
  void PublishSimTime(const gazebo::common::UpdateInfo & _info);

  /// Step control after every world update done.
  void UpdateEnd(void);
    
  /// Callback from ROS service to enable/disable step control.
  /// \param[in] req SetBool request
  /// \param[out] res SetBool response
  void OnUpdateControl(
    std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res);

  /// To enable/disable step control.
  /// \param[in] control_status
  void UpdateControl(bool step_control_status);

  /// Callback from ROS service for step control.
  /// \param[in] req StepControl request
  /// \param[out] res StepControl response
  void OnStepControl(
    utility_ros2_interface::srv::StepControl::Request::SharedPtr req,
    utility_ros2_interface::srv::StepControl::Response::SharedPtr res);

  void OpenSharedMemory(void);

  void CloseSharedMemory(void);

  /// \brief Keep a pointer to the world.
  gazebo::physics::WorldPtr world_;

  /// Gazebo-ROS node
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Publish step complete event
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr step_complete_pub_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr  enablecontrol_service_;

  /// ROS service to handle requests to unpause physics.
  rclcpp::Service<utility_ros2_interface::srv::StepControl>::SharedPtr  stepcontrol_service_;

    /// Connection to world update event, called at every iteration
  gazebo::event::ConnectionPtr world_update_event_;

  /// Connection to world update end event, called at every iteration
  gazebo::event::ConnectionPtr world_update_end_event_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_event_;

  /// Holds step control status
  bool step_control_status_;

  /// Number of steps to execute
  int64_t steps_to_execute_;

  /// If the service call to be blocked untill all steps executed
  bool step_blocking_call_;

  int m_fd;
  char *m_map;
  int i;

};


GazeboStepControl::~GazeboStepControl()
{
  CloseSharedMemory();
}

//Experimental
void GazeboStepControl::OpenSharedMemory(void)
{

    m_fd = shm_open(STORAGE_ID, O_RDWR, 0);
    if (m_fd < 0) {
        // failed to open existing file, try to create a new one
        m_fd = shm_open(STORAGE_ID, O_RDWR | O_CREAT, 0666);
        if (m_fd < 0 || ftruncate(m_fd, SHARED_MEMORY_SIZE) != 0) {
            return;
        }
    }
    m_map = static_cast<char*>(
        mmap(NULL, SHARED_MEMORY_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, m_fd, 0));
}

//Experimental
void GazeboStepControl::CloseSharedMemory(void)
{
    if (m_map != NULL) {
      int res = munmap(m_map, SHARED_MEMORY_SIZE);
      m_fd = shm_unlink(STORAGE_ID);
    }

}

void GazeboStepControl::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  world_ = _world;
  ros_node_ = gazebo_ros::Node::Get(_sdf);
  
  enablecontrol_service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
    "step_control_enable",
    std::bind(
      &GazeboStepControl::OnUpdateControl, this,
      std::placeholders::_1, std::placeholders::_2));

  stepcontrol_service_ = ros_node_->create_service<utility_ros2_interface::srv::StepControl>(
    "step",
    std::bind(
      &GazeboStepControl::OnStepControl, this,
      std::placeholders::_1, std::placeholders::_2));

  // Offer transient local durability on the clock topic so that if publishing is infrequent (e.g.
  // the simulation is paused), late subscribers can receive the previously published message(s).
  step_complete_pub_ = ros_node_->create_publisher<std_msgs::msg::Empty>(
    "/step_completed",
    rclcpp::QoS(rclcpp::KeepLast(10)).transient_local());

  // Step control flag in plugin sdf
  auto control_status_sdf = _sdf->Get<bool>("enable_control", false).first;\

  // Step control parameter
  auto enable_control_param = ros_node_->declare_parameter(
    "enable_control",
    rclcpp::ParameterValue(control_status_sdf));

  OpenSharedMemory();
  UpdateControl(enable_control_param.get<bool>());
}


GazeboStepControl::GazeboStepControl()
: step_control_status_(false), steps_to_execute_(0), step_blocking_call_(false)
{
}

void GazeboStepControl::UpdateEnd(void)
{
  if (step_control_status_ == true) {
    steps_to_execute_--;
    if (steps_to_execute_ <= 0) {
      world_->SetPaused(true);

      // publish completion topic only for non blocking service call
      if (!step_blocking_call_)
        step_complete_pub_->publish(std_msgs::msg::Empty());
    }
  }
  m_map[0] = i++;
}

void GazeboStepControl::UpdateControl(bool step_control_status)
{
  // Delete existing connection (if any)
  world_update_end_event_.reset();
  step_control_status_ = step_control_status;

  if (step_control_status_ == true) {
    world_update_end_event_ = gazebo::event::Events::ConnectWorldUpdateEnd(
    std::bind(&GazeboStepControl::UpdateEnd, this));
  }
  else {
    world_->SetPaused(false);
  }
}

void GazeboStepControl::OnUpdateControl(
  std_srvs::srv::SetBool::Request::SharedPtr _req,
  std_srvs::srv::SetBool::Response::SharedPtr _res)
{
  UpdateControl(_req->data);
 _res->success = true;
}

void GazeboStepControl::OnStepControl(
  utility_ros2_interface::srv::StepControl::Request::SharedPtr _req,
  utility_ros2_interface::srv::StepControl::Response::SharedPtr _res)
{
  steps_to_execute_ = _req->steps;
  step_blocking_call_ = _req->block;
  // Unpause physics on each step service call
  if (steps_to_execute_ > 0) {
    world_->SetPaused(false);
    if (step_blocking_call_) {
      while(steps_to_execute_ > 0)
         usleep(1000);
    }
  }
  _res->success = true;
}

GZ_REGISTER_WORLD_PLUGIN(GazeboStepControl)