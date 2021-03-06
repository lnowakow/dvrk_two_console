/*
 * Author: Lukasz Nowakowski
 * Created on: 2021-09-20
 */

#ifndef DVRK_TWO_CONSOLE_INCLUDE_STPTELEOPERATIONCURSOR_H_
#define DVRK_TWO_CONSOLE_INCLUDE_STPTELEOPERATIONCURSOR_H_

#include "stpJsonParser.h"
#include "stpStateMachine.h"
#include "stpMatrix.h"

//#include <ros/ros.h>
//#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
//#include <tf2/convert.h>
//#include <tf2_eigen/tf2_eigen.h>
//#include <Eigen/Geometry>
//#include <string>
//#include <iostream>

#include <crtk_msgs/operating_state.h>
#include <crtk_msgs/StringStamped.h>

#define waitForMessage ros::topic::waitForMessage
#define timeOut ros::Duration(0.002)
#define STATE_QUEUE 5
#define PARAM_QUEUE 5
#define COMMAND_QUEUE 20

using stpWrite = ros::Publisher;
using stpRead = ros::Subscriber;

class stpTeleOperationCursor {
 public:
  stpTeleOperationCursor();
  stpTeleOperationCursor(const std::string &name);
  //stpTeleOperationCursor(std::string json_file);
  ~stpTeleOperationCursor();

  void Startup(void);
  void Run(void);

  void set_scale(const double& scale);
  void set_registration_rotation(const Eigen::Matrix3d& rotation);
  void lock_rotation(const bool& lock);
  void lock_translation(const bool& lock);
  void set_align_mtm(const bool& alignMTM);
  void Freeze(void);

  void Init(const std::string &filename,
            const std::string &mtmName,
            const std::string &cursorName,
            const Eigen::Isometry3d &baseframe);

  void state_command(const std::string& command);

  inline std::string getTeleopName() {
    return mName;
  }

  inline std::string getMTMName() {
    return MTM;
  }

  inline std::string getCURSORName() {
    return CURSOR;
  }

  inline const bool & Selected(void) const {
    return mSelected;
  }

  inline void SetSelected(const bool selected) {
    mSelected = selected;
  }

 protected:

  void Clutch(const bool& clutch);

  struct {
    struct {
      std::string current_state;
      std::string desired_state;
      std::string following;
    } topicName;

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
    } topicName;

    stpWrite scale;
    stpWrite rotation_locked;
    stpWrite translation_locked;
    stpWrite align_mtm;

    std_msgs::Float64 m_scale;
    std_msgs::Bool m_rotation_locked;
    std_msgs::Bool m_translation_locked;
    std_msgs::Bool m_align_mtm;
  } ConfigurationEvents;

  void SetDesiredState(const std::string& state);


  stpMatrix UpdateAlignOffset(void);
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
      std::string move_cp;
      std::string servo_jp;
      std::string setpoint_js;
      std::string gripper_measured_js;
      std::string gripper_closed;
      std::string lock_orientation;
      std::string unlock_orientation;
      std::string servo_cf_body;
      std::string use_gravity_compensation;
      std::string operating_state;
      std::string state_command;
    } topicName;

    stpRead measured_cp;
    stpRead setpoint_cp;
    stpWrite move_cp;
    stpWrite servo_jp;
    stpRead setpoint_js;
    stpRead gripper_measured_js;
    stpRead gripper_closed;
    stpWrite lock_orientation;
    stpWrite unlock_orientation;
    stpWrite servo_cf_body;
    stpWrite use_gravity_compensation;

    stpRead operating_state;
    stpWrite state_command;

    sensor_msgs::JointState m_setpoint_js;
    sensor_msgs::JointState m_gripper_measured_js;
    std_msgs::Bool m_gripper_closed;
    stpMatrix m_measured_cp;
    stpMatrix m_setpoint_cp;
    geometry_msgs::TransformStamped m_move_cp;
    crtk_msgs::operating_state m_operating_state;
    stpMatrix CartesianInitial;
  } mMTM;

  struct {
    struct {
      std::string measured_cp;
      std::string servo_cp;
      std::string cursor_clicked;
      std::string operating_state;
      std::string state_command;
    } topicName;

    stpRead measured_cp;
    stpWrite servo_cp;
    stpRead cursor_clicked;

    stpRead operating_state;
    stpWrite state_command;

    stpMatrix m_measured_cp;
    geometry_msgs::TransformStamped m_servo_cp;
    crtk_msgs::operating_state m_operating_state;
    std_msgs::Bool m_cursor_clicked;
    stpMatrix CartesianInitial;
    std::string m_cursor_type;
  } mCURSOR;

  struct {
    struct {
      std::string measured_cp;
    } topicName;

    bool isValid;

    stpMatrix m_measured_cp;
    stpMatrix CartesianInitial;
  } mBASEFRAME;

  Eigen::Matrix3d m_registration_rotation;
  Eigen::Matrix3d m_alignment_offset, m_alignment_offset_initial;
  Eigen::Matrix3d mMTMClutchOrientation;

  struct {
    struct {
      std::string clutch;
    } topicName;

    stpRead clutch;
    sensor_msgs::Joy m_clutch;

    double orientation_tolerance = 3.0;
    double roll_min;
    double roll_max;
    double roll_threshold = 5.0;
    double gripper_min;
    double gripper_max;
    double gripper_threshold = 5.0;
    bool is_active = false;
    bool was_active_before_clutch = false;
  } mOPERATOR;

  bool m_back_from_clutch = false;
  bool m_following;
  bool mSelected;

  void set_following(const bool following);

  stpStateMachine mTeleopState;

 private:
  std::string mName = "stpConsole1";
  ros::NodeHandle nh;
  stpJsonParser parser;
  std::string MTM, CURSOR;

  // mMTM Callbacks
  void mtm_measured_cp_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void mtm_setpoint_cp_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void mtm_gripper_measured_js_cb(const sensor_msgs::JointState::ConstPtr& msg);
  void mtm_gripper_closed_cb(const std_msgs::Bool::ConstPtr& msg);
  void mtm_operating_state_cb(const crtk_msgs::operating_state::ConstPtr& msg);
  void mtm_setpoint_js_cb(const sensor_msgs::JointState::ConstPtr& msg);
  // mCURSOR Callbacks
  void cursor_measured_cp_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
  void cursor_clicked_cb(const std_msgs::Bool::ConstPtr& msg);
  void cursor_operating_state_cb(const crtk_msgs::operating_state::ConstPtr& msg);
  // mOPERATOR Callbacks
  void operator_clutch_cb(const sensor_msgs::Joy::ConstPtr& msg);

};

#endif //DVRK_TWO_CONSOLE_INCLUDE_STPTELEOPERATIONCURSOR_H_
