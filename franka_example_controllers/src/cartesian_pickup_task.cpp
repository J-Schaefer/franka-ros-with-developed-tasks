// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// Grasp includes (Khesbak)
#include <controller_interface/controller_base.h>
#include <franka/gripper.h>
#include <franka_example_controllers/cartesian_pickup_task.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <thread>

#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/GraspActionGoal.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/HomingGoal.h>
#include <franka_gripper/franka_gripper.h>

#include <actionlib/client/terminal_state.h>

namespace franka_example_controllers {
double xx, yy, zz;
double x_home, y_home, z_home, cnt1 = 0, cnt2 = 0;

bool CartesianPickupTask::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();

  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianPickupTask: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianPickupTask: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPickupTask: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  // ************************************************************************************************
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianPickupTask: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "CartesianPickupTask: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianPickupTask: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void CartesianPickupTask::starting(const ros::Time& /* time */) {
  //   You can initialize any thing here.
}
double schritt = 1;
double Pcheck = 0, ii;
// actionlib::SimpleActionClient<franka_gripper::HomingAction> ac("franka_gripper/homing",true);
actionlib::SimpleActionClient<franka_gripper::GraspAction> ac("franka_gripper/grasp", true);

void CartesianPickupTask::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  std::array<double, 16> new_pose;
  double x, y, z, angle, delta_x, delta_y, delta_z, ampl, f;

  if (schritt == 1) {
    x = 0.5;
    y = -0.45;
    z = 0.13;
    if (Pcheck == 0) {
      initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      elapsed_time_ = ros::Duration(0.0);
      Pcheck = 1;
      new_pose = initial_pose_;
      xx = new_pose[12];
      yy = new_pose[13];
      zz = new_pose[14];
      x_home = xx;
      y_home = yy;
      z_home = zz;
    }
    elapsed_time_ += period;
    f = 1;
    angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec() / f));
    delta_x = (x - xx) * std::sin(angle);
    delta_y = (y + yy) * std::sin(angle);
    delta_z = (z - zz) * std::sin(angle);
    ampl = std::sin(angle);
    new_pose = initial_pose_;
    new_pose[12] += delta_x;
    new_pose[13] += delta_y;
    new_pose[14] += delta_z;

    cartesian_pose_handle_->setCommand(new_pose);
    if (ampl == 1) {
      schritt = 1.5;
      Pcheck = 0;
    }
  }
  if (schritt == 1.5) {
    if (Pcheck == 0) {
      initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      elapsed_time_ = ros::Duration(0.0);
      Pcheck = 1;
      xx = new_pose[12];
      yy = new_pose[13];
      zz = new_pose[14];
    }
    z = 0.09;
    elapsed_time_ += period;
    f = 1;

    angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec() / f));
    delta_z = (z - zz) * std::sin(angle);
    ampl = std::sin(angle);
    new_pose = initial_pose_;
    new_pose[14] += delta_z;
    cartesian_pose_handle_->setCommand(new_pose);
    
    if (ampl == 1) {
      schritt = 2;
    }
  }
  if (schritt == 2) {
    franka_gripper::GraspGoal goal;
    goal.width = 0.038;
    goal.speed = 0.1;
    goal.force = 60;
    goal.epsilon.inner = 0.05;
    goal.epsilon.outer = 0.05;
    ac.sendGoal(goal);
    schritt = 2.5;

    // actionlib::SimpleActionClient<franka_gripper::StopAction> acs("franka_gripper/stop", true);
    // actionlib::SimpleActionClient<franka_gripper::MoveAction> acm("franka_gripper/move", true);
    
  }
  if (schritt == 2.5) {  //   grasp delay
    cnt1 = cnt1 + 1;
    if (cnt1 == 1000) {
      schritt = 3;
      Pcheck = 0;
    }
  }

  if (schritt == 3) {
    if (Pcheck == 0) {
      initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      elapsed_time_ = ros::Duration(0.0);
      Pcheck = 1;
      xx = new_pose[12];
      yy = new_pose[13];
      zz = new_pose[14];
    }
    z = 0.35;
    elapsed_time_ += period;
    f = 1;

    angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec() / f));
    delta_z = (z - zz) * std::sin(angle);
    ampl = std::sin(angle);
    new_pose = initial_pose_;
    new_pose[14] += delta_z;
    cartesian_pose_handle_->setCommand(new_pose);
    if (ampl == 1) {
      schritt = 4;
      Pcheck = 0;
    }
  }
  if (schritt == 4) {
    if (Pcheck == 0) {
      initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      elapsed_time_ = ros::Duration(0.0);
      Pcheck = 1;
      xx = new_pose[12];
      yy = new_pose[13];
      zz = new_pose[14];
    }
    x = 0.6;
    y = 0;
    z = 0.09;
    elapsed_time_ += period;
    f = 1;
    angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec() / f));
    delta_x = (x - xx) * std::sin(angle);
    delta_y = (y - yy) * std::sin(angle);
    delta_z = (z - zz) * std::sin(angle);
    ampl = std::sin(angle);
    new_pose = initial_pose_;
    new_pose[12] += delta_x;
    new_pose[13] += delta_y;
    new_pose[14] += delta_z;
    cartesian_pose_handle_->setCommand(new_pose);
    if (ampl == 1) {
      bool success = ac.isServerConnected();  // will wait for infinite time
      franka_gripper::GraspGoal goal;
      goal.width = 0.08;
      goal.speed = 0.1;
      ac.sendGoal(goal);
      schritt = 5;
      Pcheck = 0;
    }
  }

  if (schritt == 5) {  // Back to  H o m e
    if (Pcheck == 0) {
      initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      elapsed_time_ = ros::Duration(0.0);
      Pcheck = 1;
      xx = new_pose[12];
      yy = new_pose[13];
      zz = new_pose[14];
    }
    elapsed_time_ += period;
    f = 1;
    angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec() / f));
    delta_x = (x_home - xx) * std::sin(angle);
    delta_y = (y_home - yy) * std::sin(angle);
    delta_z = (z_home - zz) * std::sin(angle);
    ampl = std::sin(angle);
    new_pose = initial_pose_;
    new_pose[12] += delta_x;
    new_pose[13] += delta_y;
    new_pose[14] += delta_z;
    cartesian_pose_handle_->setCommand(new_pose);
    if (ampl == 1) {
      schritt = 6;  //  set schritt = 1  for periodic scenario
      Pcheck = 0;
      cnt1 =0;

      // exit(0);
    }
  }

}  

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPickupTask,
                       controller_interface::ControllerBase)
