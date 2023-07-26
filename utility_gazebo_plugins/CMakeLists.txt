cmake_minimum_required(VERSION 3.5)
project(utility_gazebo_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(utility_ros2_interface REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sdformat9 REQUIRED)

link_directories(
  ${gazebo_dev_LIBRARY_DIRS}
)

# gazebo_ros_camera
add_library(gazebo_ros_camera_jpg SHARED
  src/gazebo_ros_camera_jpg.cpp
)
target_include_directories(gazebo_ros_camera_jpg PUBLIC include)
ament_target_dependencies(gazebo_ros_camera_jpg
  "camera_info_manager"
  "gazebo_dev"
  "gazebo_ros"
  "gazebo_plugins"
  "image_transport"
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
  "std_msgs"
)

target_link_libraries(gazebo_ros_camera_jpg
  CameraPlugin
  DepthCameraPlugin
  ${GAZEBO_LIBRARIES}
)

find_library(MULTI_CAMERA_PLUGIN_LIBRARY NAMES multi_camera_plugin PATHS ${AMENT_PREFIX_PATH}/lib)
if(MULTI_CAMERA_PLUGIN_LIBRARY)
    target_link_libraries(gazebo_ros_camera_jpg ${MULTI_CAMERA_PLUGIN_LIBRARY})
endif()

ament_export_libraries(gazebo_ros_camera_jpg)

# gazebo_sharedmem_camera
add_library(gazebo_ros_camera_sharedmem SHARED
  src/gazebo_ros_camera_sharedmem.cpp
)
target_include_directories(gazebo_ros_camera_sharedmem PUBLIC include)
ament_target_dependencies(gazebo_ros_camera_sharedmem
  "camera_info_manager"
  "gazebo_dev"
  "gazebo_ros"
  "gazebo_plugins"
  "image_transport"
  "rclcpp"
  "sensor_msgs"
  "OpenCV"
  "cv_bridge"
  "std_msgs"
)

link_directories(${GAZEBO_LIBRARY_DIRS})

target_link_libraries(gazebo_ros_camera_sharedmem
  CameraPlugin
  DepthCameraPlugin
  ${GAZEBO_LIBRARIES}
)

find_library(MULTI_CAMERA_PLUGIN_LIBRARY NAMES multi_camera_plugin PATHS ${AMENT_PREFIX_PATH}/lib)
if(MULTI_CAMERA_PLUGIN_LIBRARY)
    target_link_libraries(gazebo_ros_camera_sharedmem ${MULTI_CAMERA_PLUGIN_LIBRARY})
endif()

ament_export_libraries(gazebo_ros_camera_sharedmem)

set(conveyorbelt_plugin_name ConveyorBeltPlugin)
add_library(${conveyorbelt_plugin_name} SHARED src/ConveyorBeltPlugin.cpp)

target_include_directories(${conveyorbelt_plugin_name}  PUBLIC include)
ament_target_dependencies(${conveyorbelt_plugin_name}
  "gazebo_ros"
  "gazebo_dev"
  "utility_ros2_interface"
  "rclcpp"
  "std_msgs"
  "std_srvs"
)

ament_export_libraries(${conveyorbelt_plugin_name})

add_library(gazebo_step_control SHARED
  src/gazebo_step_control.cpp
)
ament_target_dependencies(gazebo_step_control
  "utility_ros2_interface"
  "gazebo_dev"
  "rclcpp"
  "gazebo_ros"
  "std_srvs"
)

ament_export_libraries(gazebo_step_control)
ament_export_dependencies(gazebo_dev)

ament_package()


install(
  PROGRAMS
    DESTINATION lib/${PROJECT_NAME}/
)

install(
  TARGETS
  gazebo_ros_camera_jpg
  gazebo_ros_camera_sharedmem
  gazebo_step_control
  ${conveyorbelt_plugin_name}
  LIBRARY DESTINATION lib
  )
