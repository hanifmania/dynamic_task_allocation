#ifndef GAZEBO_INFO_H
#define GAZEBO_INFO_H

/// \brief compile the code with Gazebo 8.0, there are many warning about "-Wdeprecated-declarations", ignored them
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

//#include <functional>
#include <algorithm>
#include <assert.h>
#include <vector_angle.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <sdf/sdf.hh>

#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <allocation_common/gazebo2world_info.h>
#include <allocation_common/gazebo_robot_info.h>
#include <allocation_common/gazebo_task_info.h>
#include <allocation_common/allocation_task_info.h>
#include <allocation_common/robot2gazebo_info.h>
#include <allocation_common/terminal2gazebo_info.h>
#include <allocation_common/terminal2robot_info.h>
#include <dynamic_reconfigure/server.h>
//#include <allocation_gazebo/Robot_GazeboConfig.h>
#include <Core.hpp>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#define PI 3.14159265
#define CM2M_CONVERSION 0.01
#define M2CM_CONVERSION 100

#define XUNIT_VECTOR ignition::math::Vector3d::UnitX
#define YUNIT_VECTOR ignition::math::Vector3d::UnitY
#define ZERO_VECTOR  ignition::math::Vector3d::Zero
#define ZERO_QUATER  ignition::math::Quaterniond(0,0,0,0)

const std::string robot_name="Robot";
const std::string task_name="Task";
const double g = 9.8;

// robot_state
struct Pose
{
    ignition::math::Vector3d    position;
    ignition::math::Quaterniond orient;
};

struct Twist
{
    ignition::math::Vector3d    linear;
    ignition::math::Vector3d    angular;
};

struct Robot_state
{
    std::string      robot_name;
    int              robot_ID;
    Pose             pose;
    Twist            twist;
    Robots_mode      current_mode;
    std::string      reference_frame;
};

// task_state
struct Task_state
{
    DPoint           world_pos;
    bool             istarget;
    bool             isvalid;
};

#endif //! GAZEBO_INFO_H
