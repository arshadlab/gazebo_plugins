// Copyright 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_CAMERA_JPG_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_CAMERA_JPG_HPP_

#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo_plugins/multi_camera_plugin.hpp>
#include <std_msgs/msg/empty.hpp>

#include <memory>
#include <string>

class GazeboRosCameraJPGPrivate;

/// A plugin that publishes raw images and camera info for generic camera sensors.
/// It can also be configured to publish raw depth images, point cloud
/// and camera info for depth camera sensors.
/// Also configurable as multi camera sensor.
/**
  Example Usage:
  \code{.xml}
    <plugin name="plugin_name" filename="libgazebo_ros_camera.so">
      <!-- Change namespace, camera name and topics so:
           * Images are published to: /custom_ns/custom_camera/custom_image
           * Camera info is published to: /custom_ns/custom_camera/custom_info
           * Trigger is received on: /custom_ns/custom_camera/custom_trigger
      -->
      <ros>
        <namespace>custom_ns</namespace>
        <remapping>image_raw:=custom_img</remapping>
        <remapping>camera_info:=custom_info</remapping>
        <remapping>image_trigger:=custom_trigger</remapping>
      </ros>

      <!-- Set camera name. If empty, defaults to sensor name -->
      <camera_name>custom_camera</camera_name>

      <!-- Set TF frame name. If empty, defaults to link name -->
      <frame_name>custom_frame</frame_name>

      <!-- Set to true to turn on triggering -->
      <triggered>true</triggered>

      <hack_baseline>0.07</hack_baseline>
    </plugin>
  \endcode
*/
class GazeboRosCameraJPG
  : public gazebo::CameraPlugin, gazebo::DepthCameraPlugin, gazebo_plugins::MultiCameraPlugin
{
public:
  /// Constructor
  GazeboRosCameraJPG();

  /// Destructor
  ~GazeboRosCameraJPG();

protected:
  // Documentation inherited
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

  /// Helper to process and publish the image received to appropriate topic.
  /*
   * \param[in] _image Image to publish
   * \param[in] _width Image width
   * \param[in] _height Image height
   * \param[in] camera_num Index number of camera
   */
  void NewFrame(
    const unsigned char * _image,
    unsigned int _width, unsigned int _height,
    int camera_num);

  /// Callback when camera produces a new image.
  /*
   * \details This is called at the camera's update rate.
   * \details Not called when the camera isn't active. For a triggered camera, it will only be
   * called after triggered.
   * \param[in] _image Image
   * \param[in] _width Image width
   * \param[in] _height Image height
   * \param[in] _depth Image depth
   * \param[in] _format Image format
   */
  void OnNewFrame(
    const unsigned char * _image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string & _format) override;

  /// Callback when depth camera produces a new image.
  /*
  * \details This is called at the camera's update rate.
  * \details Not called when the camera isn't active. For a triggered camera, it will only be
  * called after triggered.
  * \param[in] _image Image
  * \param[in] _width Image width
  * \param[in] _height Image height
  * \param[in] _depth Image depth
  * \param[in] _format Image format
  */
  void OnNewImageFrame(
    const unsigned char * _image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string & _format) override;

  /// Callback when camera produces a new depth image.
  /*
  * \details This is called at the camera's update rate.
  * \details Not called when the camera isn't active. For a triggered camera, it will only be
  * called after triggered.
  * \param[in] _image Image
  * \param[in] _width Image width
  * \param[in] _height Image height
  * \param[in] _depth Image depth
  * \param[in] _format Image format
  */
  void OnNewDepthFrame(
    const float * _image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string & _format) override;


  /// Callback when multi camera produces a new image.
  /*
  * \details This is called at the multi camera's update rate.
  * \details Not called when the camera isn't active. For a triggered multi camera, it will only be
  * called after triggered.
  * \param[in] _image Image
  * \param[in] _width Image width
  * \param[in] _height Image height
  * \param[in] _depth Image depth
  * \param[in] _format Image format
  * \param[in] _camera_num Index number of camera
  */
  void OnNewMultiFrame(
    const unsigned char * _image,
    unsigned int _width, unsigned int _height,
    unsigned int _depth, const std::string & _format, const int _camera_num) override;

  /// Callback when camera is triggered.
  void OnTrigger(const std_msgs::msg::Empty::SharedPtr _dummy);

  /// Callback on pre-render event.
  void PreRender();

  /// Enables or disables the camera so it produces messages or not.
  /// param[in] _enabled True to enable.
  void SetCameraEnabled(const bool _enabled);

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosCameraJPGPrivate> impl_;

  // A handler for the param change callback.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_change_callback_handler_;
};


#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_CAMERA_JPG_HPP_
