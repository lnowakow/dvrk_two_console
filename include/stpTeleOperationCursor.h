/*
 * Author: Lukasz Nowakowski
 * Created on: 2021-09-20
 */

#ifndef DVRK_TWO_CONSOLE_INCLUDE_STPTELEOPERATIONCURSOR_H_
#define DVRK_TWO_CONSOLE_INCLUDE_STPTELEOPERATIONCURSOR_H_

#include "stpStateMachine.h"

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>
#include <string>
#include <iostream>

#include <crtk_msgs/operating_state.h>

#define waitForMessage ros::topic::waitForMessage
#define timeOut ros::Duration(0.002)

using stpWrite = ros::Publisher;
using stpRead = ros::Subscriber;
using stpVoid = ros::Subscriber;

class stpTeleOperationCursor {
 public:
  stpTeleOperationCursor();
  ~stpTeleOperationCursor();

  void Startup(void);
  void Run(void);

  void set_scale(const double& scale);
  void set_registration_rotation(const Eigen::Matrix3d& rotation);
  void lock_rotation(const bool& lock);
  void lock_translation(const bool& lock);
  void set_align_mtm(const bool& alignMTM);

 protected:

  void Init(void);

  struct {
    stpWrite current_state;
    stpWrite desired_state;
    stpWrite following;
  } MessageEvents;

  struct {
    struct {
      std::string scale;
      std::string rotation_locked;
      std::string translation_locked;
      std::string align_mtm;
    } topicNames;

    stpWrite scale;
    stpWrite rotation_locked;
    stpWrite translation_locked;
    stpWrite align_mtm;

    std_msgs::Float64 m_scale;
    sensor_msgs::Joy m_rotation_locked;
    sensor_msgs::Joy m_translation_locked;
    std_msgs::Bool m_align_mtm;
    std_msgs::Bool m_following;
  } ConfigurationEvents;

  void SetDesiredState(const std::string& state);
  void state_command(const std::string& command);

  Eigen::Matrix3d UpdateAlignOffset(void);
  void UpdateInitialState(void);


  void StateChanged(void);
  void RunAllStates(void);
  void TransitionDisabled(void);
  void EnterSettingArmsState(void);
  void TransitionSettingArmsState(void);
  void EnterAligningMTM(void);
  void RunAligningMTM(void);
  void TransitionAligningMTM(void);
  void EnterEnabled(void);
  void RunEnabled(void);
  void TransitionEnabled(void);

  struct {
    struct {
      std::string measured_cp;
      std::string setpoint_cp;
      std::string gripper_measured_js;
      std::string gripper_closed;
      std::string operating_state;
    } topicName;

    stpRead measured_cp;
    stpRead setpoint_cp;
    stpWrite move_cp;
    stpRead gripper_measured_js;
    stpRead gripper_closed;
    stpWrite lock_orientation;
    stpVoid unlock_orientation;
    stpWrite servo_cf_body;

    stpRead operating_state;
    stpWrite state_command;

    sensor_msgs::JointState m_gripper_measured_js;
    std_msgs::Bool m_gripper_closed;
    geometry_msgs::TransformStamped m_measured_cp;
    geometry_msgs::TransformStamped m_setpoint_cp;
    geometry_msgs::TransformStamped m_move_cp;
    crtk_msgs::operating_state m_operating_state;
    Eigen::Matrix4d CartesianInitial;
  } mMTM;

  struct {
    struct {
      std::string measured_cp;
      std::string cursor_clicked;
      std::string operating_state;
    } topicName;

    stpRead measured_cp;
    stpWrite servo_cp;
    stpRead cursor_clicked;

    stpRead operating_state;
    stpWrite state_command;

    geometry_msgs::TransformStamped m_measured_cp;
    geometry_msgs::TransformStamped m_servo_cp;
    crtk_msgs::operating_state m_operating_state;
    std_msgs::Bool m_cursor_clicked;
    Eigen::Matrix4d CartesionInitial;
    std::string m_cursor_type;
  } mCURSOR;

  struct {
    struct {
      std::string measured_cp;
    } topicName;

    bool isValid;

    stpRead measured_cp;
    geometry_msgs::TransformStamped m_measured_cp;
    Eigen::Matrix4d CartesianInitial;
  } mBASEFRAME;

  struct {
    stpRead operator_present;
    stpRead clutch;

    std_msgs::Bool m_operator_present;
    std_msgs::Bool m_clutch;
  } mFOOTPEDALS;

  Eigen::Matrix3d m_registration_rotation;
  Eigen::Matrix3d m_alignment_offset;
  Eigen::Matrix3d mMTMClutchOrientation;

  struct {
    bool is_active = false;
    bool was_active_before_clutch = false;
  } mOPERATOR;

  bool m_back_from_clutch = false;

  void set_following(const bool following);

  stpStateMachine mTeleopState;

 private:
  std::string mName;
  ros::NodeHandle nh;

};

#endif //DVRK_TWO_CONSOLE_INCLUDE_STPTELEOPERATIONCURSOR_H_
