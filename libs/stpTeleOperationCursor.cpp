/*
 * Author: Lukasz Nowakowski
 * Created on: 2021-09-20
 */

#include "../include/stpTeleOperationCursor.h"

stpTeleOperationCursor::stpTeleOperationCursor():
mTeleopState(mName, "DISABLED")
{
  Init();
}

stpTeleOperationCursor::~stpTeleOperationCursor() {
}

void stpTeleOperationCursor::Init() {
  // Configure the state machine
  mTeleopState.AddState("SETTING_ARMS_STATE");
  mTeleopState.AddState("ALIGNING_MTM");
  mTeleopState.AddState("ENABLED");
  mTeleopState.AddAllowedDesiredState("ENABLED");
  mTeleopState.AddAllowedDesiredState("ALIGNING_MTM");
  mTeleopState.AddAllowedDesiredState("DISABLED");

  // State change, to convert to string events for users
  mTeleopState.SetStateChangedCallback(&stpTeleOperationCursor::StateChanged, this);
  // Run for all states
  mTeleopState.SetRunCallback(&stpTeleOperationCursor::RunAllStates, this);

}

void stpTeleOperationCursor::StateChanged(void) {
  const std::string newState = mTeleopState.CurrentState();
  std_msgs::String stateToPublish;
  stateToPublish.data = newState;
  MessageEvents.current_state.publish(newState);
}

void stpTeleOperationCursor::RunAllStates(void) {
  geometry_msgs::TransformStampedConstPtr executionResult;
  executionResult = waitForMessage<geometry_msgs::TransformStamped>(mMTM.topicName.measured_cp,
                                                                    timeOut);
  // Get MTM Cartesion Position
  if (executionResult == nullptr) {
    ROS_INFO("%s: unable to get cartesian position from MTM", this->mName.c_str());
    mTeleopState.SetDesiredState("DISABLED");
  }
  executionResult = waitForMessage<geometry_msgs::TransformStamped>(mMTM.topicName.setpoint_cp,
                                                                    timeOut);
  if (executionResult == nullptr) {
    ROS_INFO("%s: unable to get cartesian setpoint from MTM", this->mName.c_str());
    mTeleopState.SetDesiredState("DISABLED");
  }

  // Get PSM Cartesian Position
  executionResult = waitForMessage<geometry_msgs::TransformStamped>(mCURSOR.topicName.measured_cp,
                                                                    timeOut);
  if (executionResult == nullptr) {
    ROS_INFO("%s: unable to get cartesian position from CURSOR", this->mName.c_str());
    mTeleopState.SetDesiredState("DISABLED");
  }

  // Get Base-Frame Cartesian Position if Available
  if (mBASEFRAME.isValid) {
    executionResult = waitForMessage<geometry_msgs::TransformStamped>(mBASEFRAME.topicName.measured_cp,
                                                                      timeOut);
    if (executionResult == nullptr) {
      ROS_INFO("%s: unable to get cartesian position from BASEFRAME", this->mName.c_str());
      mTeleopState.SetDesiredState("DISABLED");
    }
  }

  // Check if anyone wanted to disable anyway
  if ((mTeleopState.DesiredState() == "DISABLED") && (mTeleopState.CurrentState() != "DISABLED")) {
    set_following(false);
    mTeleopState.SetCurrentState("DISABLED");
    return;
  }

  // Monitor state of arms if needed
  if ((mTeleopState.CurrentState() != "DISABLED") && (mTeleopState.CurrentState() != "SETTING_ARMS_STATE")) {
    if (mCURSOR.m_operating_state.state != "ENABLED" || !mCURSOR.m_operating_state.is_homed) {
      ROS_INFO("%s: CURSOR is not in state \"ENABLED\" anrmore", this->mName.c_str());
      mTeleopState.SetDesiredState("DISABLED");
    }
    if (mMTM.m_operating_state.state != "ENABLED" || !mMTM.m_operating_state.is_homed) {
      ROS_INFO("%s: MTM is not in state \"ENABLED\" anrmore", this->mName.c_str());
      mTeleopState.SetDesiredState("DISABLED");
    }
  }
}

void stpTeleOperationCursor::TransitionDisabled(void) {
  if (mTeleopState.DesiredStateIsNotCurrent()) {
    mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
  }
}

void stpTeleOperationCursor::EnterSettingArmsState(void) {
  // Request state if needed
  if (mCURSOR.m_operating_state.state != "ENABLED") {
    mCURSOR.state_command.publish(std::string("enable"));
  }
  if (mCURSOR.m_operating_state.is_homed) {
    mCURSOR.state_command.publish(std::string("home"));
  }

  if (mMTM.m_operating_state.state != "ENABLED") {
    mMTM.state_command.publish(std::string("enable"));
  }
  if (mMTM.m_operating_state.is_homed) {
    mMTM.state_command.publish(std::string("home"));
  }
}

void stpTeleOperationCursor::TransitionSettingArmsState(void) {
  // Check state
  if ((mCURSOR.m_operating_state.state == "ENABLED") && mCURSOR.m_operating_state.is_homed
    && (mMTM.m_operating_state.state == "ENABLED") && mMTM.m_operating_state.is_homed) {
    mTeleopState.SetCurrentState("ALIGNING_MTM");
    return;
  }

  // Check if you're ever able to align and cut out if your timeout?
  // Not implementing
}

void stpTeleOperationCursor::EnterAligningMTM(void) {
  // update scale
  ConfigurationEvents.scale.publish(ConfigurationEvents.m_scale);

  // if we don't align MTM, just stay in the same position
  if (!ConfigurationEvents.m_align_mtm.data) {
    mMTM.move_cp.publish(mMTM.m_setpoint_cp);
  }

  if (m_back_from_clutch) {
    mOPERATOR.is_active = mOPERATOR.was_active_before_clutch;
    mOPERATOR.was_active_before_clutch = false;
  }
}

void stpTeleOperationCursor::RunAligningMTM(void) {
  // if clutched or align not needed, do nothing
  if (mFOOTPEDALS.m_clutch.data || !ConfigurationEvents.m_align_mtm.data) {
    return;
  }

  // set trajectory goal periodically??, this will track PSM motion
  //Orientate MTM with PSM
  Eigen::Matrix4d mtmCartesianGoal;

}

void stpTeleOperationCursor::TransitionAligningMTM(void) {

}

void stpTeleOperationCursor::EnterEnabled(void) {

}

void stpTeleOperationCursor::RunEnabled(void) {

}

void stpTeleOperationCursor::TransitionEnabled(void) {

}
