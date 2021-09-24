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
  // Disabled
  mTeleopState.SetTransitionCallback("DISABLED", &stpTeleOperationCursor::TransitionDisabled, this);
  // Setting arms state
  mTeleopState.SetEnterCallback("SETTING_ARMS_STATE", &stpTeleOperationCursor::EnterSettingArmsState, this);
  mTeleopState.SetTransitionCallback("SETTING_ARMS_STATE", &stpTeleOperationCursor::TransitionSettingArmsState, this);
  // Aligning MTM
  mTeleopState.SetEnterCallback("ALIGNING_MTM", &stpTeleOperationCursor::EnterAligningMTM, this);
  mTeleopState.SetRunCallback("ALIGNING_MTM", &stpTeleOperationCursor::RunAligningMTM, this);
  mTeleopState.SetTransitionCallback("ALIGNING_MTM", &stpTeleOperationCursor::TransitionAligningMTM, this);
  // Enabled
  mTeleopState.SetEnterCallback("ENABLED", &stpTeleOperationCursor::EnterEnabled, this);
  mTeleopState.SetRunCallback("ENABLED", &stpTeleOperationCursor::RunEnabled, this);
  mTeleopState.SetTransitionCallback("ENABLED", &stpTeleOperationCursor::TransitionEnabled, this);

  // Set up all publishers and subscribers
  // MESSAGE EVENTS
  MessageEvents.current_state = nh.advertise<std_msgs::String>(MessageEvents.topicName.current_state, STATE_QUEUE);
  MessageEvents.desired_state = nh.advertise<std_msgs::String>(MessageEvents.topicName.desired_state, STATE_QUEUE);
  MessageEvents.following = nh.advertise<std_msgs::Bool>(MessageEvents.topicName.following, STATE_QUEUE);
  // CONFIGURATION EVENTS
  ConfigurationEvents.scale = nh.advertise<std_msgs::Float64>(ConfigurationEvents.topicName.scale, PARAM_QUEUE);
  ConfigurationEvents.rotation_locked = nh.advertise<std_msgs::Bool>(ConfigurationEvents.topicName.rotation_locked, PARAM_QUEUE);
  ConfigurationEvents.translation_locked = nh.advertise<std_msgs::Bool>(ConfigurationEvents.topicName.translation_locked, PARAM_QUEUE);
  ConfigurationEvents.align_mtm = nh.advertise<std_msgs::Bool>(ConfigurationEvents.topicName.align_mtm, PARAM_QUEUE);
  // MTM TOPICS
  mMTM.measured_cp = nh.subscribe(mMTM.topicName.measured_cp,
                                  COMMAND_QUEUE,
                                  &stpTeleOperationCursor::mtm_measured_cp_cb,
                                  this);
  mMTM.setpoint_cp = nh.subscribe(mMTM.topicName.setpoint_cp,
                                  COMMAND_QUEUE,
                                  &stpTeleOperationCursor::mtm_setpoint_cp_cb,
                                  this);
  mMTM.move_cp = nh.advertise<geometry_msgs::TransformStamped>(mMTM.topicName.move_cp,
                                                               COMMAND_QUEUE);
  mMTM.gripper_measured_js = nh.subscribe(mMTM.topicName.gripper_measured_js,
                                          COMMAND_QUEUE,
                                          &stpTeleOperationCursor::mtm_gripper_measured_js_cb,
                                          this);
  mMTM.gripper_closed = nh.subscribe(mMTM.topicName.gripper_closed,
                                     COMMAND_QUEUE,
                                     &stpTeleOperationCursor::mtm_gripper_closed_cb,
                                     this);
  mMTM.lock_orientation = nh.advertise<geometry_msgs::Quaternion>(mMTM.topicName.lock_orientation,
                                                                  PARAM_QUEUE);
  mMTM.unlock_orientation = nh.advertise<std_msgs::Empty>(mMTM.topicName.unlock_orientation,
                                                          PARAM_QUEUE);
  mMTM.servo_cf_body = nh.advertise<geometry_msgs::WrenchStamped>(mMTM.topicName.servo_cf_body,
                                                                  PARAM_QUEUE);
  mMTM.use_gravity_compensation = nh.advertise<std_msgs::Bool>(mMTM.topicName.use_gravity_compensation,
                                                               PARAM_QUEUE);
  mMTM.operating_state = nh.subscribe(mMTM.topicName.operating_state,
                                      STATE_QUEUE,
                                      &stpTeleOperationCursor::mtm_operating_state_cb,
                                      this);
  mMTM.state_command = nh.advertise<crtk_msgs::StringStamped>(mMTM.topicName.state_command,
                                                              STATE_QUEUE);
  // CURSOR TOPICS
  mCURSOR.measured_cp = nh.subscribe(mCURSOR.topicName.measured_cp,
                                     COMMAND_QUEUE,
                                     &stpTeleOperationCursor::cursor_measured_cp_cb,
                                     this);
  mCURSOR.servo_cp = nh.advertise<geometry_msgs::TransformStamped>(mCURSOR.topicName.servo_cp,
                                                                   COMMAND_QUEUE);
  mCURSOR.cursor_clicked = nh.subscribe(mCURSOR.topicName.cursor_clicked,
                                        PARAM_QUEUE,
                                        &stpTeleOperationCursor::cursor_clicked_cb,
                                        this);
  mCURSOR.operating_state = nh.subscribe(mCURSOR.topicName.operating_state,
                                         STATE_QUEUE,
                                         &stpTeleOperationCursor::cursor_operating_state_cb,
                                         this);
  mCURSOR.state_command = nh.advertise<crtk_msgs::StringStamped>(mCURSOR.topicName.state_command,
                                                                 STATE_QUEUE);
  // BASEFRAME TOPICS
  /*
   *
   */
  // OPERATOR TOPICS
  mOPERATOR.footpedal = nh.subscribe(mOPERATOR.topicName.footpedal,
                                     PARAM_QUEUE,
                                     &stpTeleOperationCursor::operator_footpedal_cb,
                                     this);
  // default ConfigurationEvents
  ConfigurationEvents.m_scale.data = 0.3;
  ConfigurationEvents.m_rotation_locked.data = false;
  ConfigurationEvents.m_translation_locked.data = false;
  ConfigurationEvents.m_align_mtm.data = true;
}

void stpTeleOperationCursor::Startup(void) {
  set_scale(ConfigurationEvents.m_scale.data);
  set_following(false);
  lock_rotation(ConfigurationEvents.m_rotation_locked.data);
  lock_translation(ConfigurationEvents.m_translation_locked.data);
  set_align_mtm(ConfigurationEvents.m_align_mtm.data);
}

void stpTeleOperationCursor::Run(void) {
  mTeleopState.Run();
}

void stpTeleOperationCursor::Clutch(const bool &clutch) {
  // if teleoperation is activated
  if (clutch) {
    // keep track of last follow mode
    mOPERATOR.was_active_before_clutch = mOPERATOR.is_active;
    set_following(false);
    stpMatrix mtmGoal;
    mtmGoal.FromNormalized(mCURSOR.m_measured_cp.GetMatrixRotation());
    mtmGoal.SetTranslation(mMTM.m_measured_cp.GetMatrixTranslation());
    mMTM.move_cp.publish(mtmGoal.transform_stamped_);
    ROS_INFO("%s: console clutch pressed", this->mName.c_str());

    // no for applied but gravity and locked orientation
    geometry_msgs::WrenchStamped wrench;
    mMTM.servo_cf_body.publish(wrench);
    std_msgs::Bool gravity;
    gravity.data = true;
    mMTM.use_gravity_compensation.publish(gravity);
    if (ConfigurationEvents.m_align_mtm.data || ConfigurationEvents.m_rotation_locked.data) {
      // lock in current position
      mMTM.lock_orientation.publish(mMTM.m_measured_cp.transform_stamped_.transform.rotation);
    } else {
      std_msgs::Empty empty;
      mMTM.unlock_orientation.publish(empty);
    }

    // asks to freeze but there's no explanation for freeze.
    //
  } else {
    mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
    m_back_from_clutch = true;
  }
}

void stpTeleOperationCursor::state_command(const std::string& command) {
  if (command == "enable") {
    SetDesiredState("ENABLED");
    return;
  }
  if (command == "disable") {
    SetDesiredState("DISABLED");
    return;
  }
  if (command == "align_mtm") {
    SetDesiredState("ALIGNING_MTM");
    return;
  }
  ROS_INFO("%s: %s doesn't seem to be a valid state_command", this->mName.c_str(), command.c_str());
}

void stpTeleOperationCursor::SetDesiredState(const std::string& state) {

  std_msgs::String desiredState;
  desiredState.data = state;

  // try to find the state in the state machine
  if (!mTeleopState.StateExists(state)) {
    ROS_ERROR("%s: unsupported state %s", this->mName.c_str(), state.c_str());
    return;
  }
  // return is already the desired state
  if (mTeleopState.DesiredState() == state) {
    MessageEvents.desired_state.publish(desiredState);
    return;
  }
  // try to set desired state
  try {
    mTeleopState.SetDesiredState(state);
  } catch (...) {
    ROS_ERROR("%s: %s is not an allowed desired state", this->mName.c_str(), state.c_str());
    return;
  }
  // force operator to indicate they are present
  mOPERATOR.is_active = false;
  MessageEvents.desired_state.publish(desiredState);
  ROS_INFO("%s: set desired state to %s", this->mName.c_str(), state.c_str());
}

stpMatrix stpTeleOperationCursor::UpdateAlignOffset(void) {
  stpMatrix desiredOrientation, mtmRotation;
  mtmRotation.SetRotation(m_registration_rotation.inverse() * mCURSOR.m_measured_cp.GetMatrixRotation());
  desiredOrientation.SetRotation(mtmRotation.Inverse() * m_alignment_offset);
  return desiredOrientation;
}

void stpTeleOperationCursor::UpdateInitialState(void) {
  mMTM.CartesianInitial.SetRotation(mMTM.m_measured_cp.GetMatrixRotation());
  mMTM.CartesianInitial.SetTranslation(mMTM.m_measured_cp.GetMatrixTranslation());
  mCURSOR.CartesianInitial.SetRotation(mCURSOR.m_measured_cp.GetMatrixRotation());
  mCURSOR.CartesianInitial.SetTranslation(mCURSOR.m_measured_cp.GetMatrixTranslation());
  UpdateAlignOffset();
  m_alignment_offset_initial = m_alignment_offset;
  if (mBASEFRAME.m_measured_cp.isValid(mBASEFRAME.topicName.measured_cp)) {
    mBASEFRAME.CartesianInitial.SetRotation(mBASEFRAME.m_measured_cp.GetMatrixRotation());
    mBASEFRAME.CartesianInitial.SetTranslation(mBASEFRAME.m_measured_cp.GetMatrixTranslation());
  }
}

void stpTeleOperationCursor::set_scale(const double& scale) {
  // set scale
  ConfigurationEvents.m_scale.data = scale;
  ConfigurationEvents.scale.publish(ConfigurationEvents.m_scale);
  // update MTM/CURSOR previous position to prevent jumps
  UpdateInitialState();
}

void stpTeleOperationCursor::set_registration_rotation(const Eigen::Matrix3d& rotation) {
  m_registration_rotation = rotation;
}

void stpTeleOperationCursor::lock_rotation(const bool& lock) {
  ConfigurationEvents.m_rotation_locked.data = lock;
  ConfigurationEvents.rotation_locked.publish(ConfigurationEvents.m_rotation_locked);
  // when releasing the orientation, MTM orientation is likely off
  // so force re-align
  if (lock == false) {
    set_following(false);
    mTeleopState.SetCurrentState("DISABLED");
  } else {
    // update MTM/CURSOR previous position
    UpdateInitialState();
    // lock orientation is the arm is running
    if (mTeleopState.CurrentState() == "ENABLED") {
      mMTM.lock_orientation.publish(mMTM.m_measured_cp.transform_stamped_.transform.rotation);
    }
  }
}

void stpTeleOperationCursor::lock_translation(const bool& lock) {
  ConfigurationEvents.m_translation_locked.data = lock;
  ConfigurationEvents.translation_locked.publish(ConfigurationEvents.m_translation_locked);
  // update MTM/CURSOR previous position
  UpdateInitialState();
}

void stpTeleOperationCursor::set_align_mtm(const bool& alignMTM) {
  geometry_msgs::Vector3ConstPtr lockValidity;
  lockValidity = waitForMessage<geometry_msgs::Vector3>(mMTM.topicName.lock_orientation, timeOut);
  std_msgs::EmptyConstPtr unlockValidity;
  unlockValidity = waitForMessage<std_msgs::Empty>(mMTM.topicName.unlock_orientation, timeOut);
  if (lockValidity != nullptr && unlockValidity != nullptr) {
    ConfigurationEvents.m_align_mtm.data = alignMTM;
  } else {
    if (alignMTM) {
      ROS_WARN("%s: unable to force MTM alignment, the device doesn't provide commands to lock/unlock orientation", this->mName.c_str());
      ConfigurationEvents.m_align_mtm.data = false;
    }
  }
  ConfigurationEvents.align_mtm.publish(ConfigurationEvents.m_align_mtm);
  // force re-align if the teleop is already enabled
  if (mTeleopState.CurrentState() == "ENABLED") {
    mTeleopState.SetCurrentState("DISABLED");
  }
}

void stpTeleOperationCursor::StateChanged(void) {
  const std::string newState = mTeleopState.CurrentState();
  crtk_msgs::StringStamped stateToPublish;
  stateToPublish.string = newState;
  MessageEvents.current_state.publish(stateToPublish);
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
  crtk_msgs::StringStamped sendState;
  // Request state if needed
  if (mCURSOR.m_operating_state.state != "ENABLED") {
    sendState.string = "enable";
    mCURSOR.state_command.publish(sendState);
  }
  if (mCURSOR.m_operating_state.is_homed) {
    sendState.string = "home";
    mCURSOR.state_command.publish(sendState);
  }

  if (mMTM.m_operating_state.state != "ENABLED") {
    sendState.string = "enable";
    mMTM.state_command.publish(sendState);
  }
  if (mMTM.m_operating_state.is_homed) {
    sendState.string = "home";
    mMTM.state_command.publish(sendState);
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
    mMTM.move_cp.publish(mMTM.m_setpoint_cp.transform_stamped_);
  }

  if (m_back_from_clutch) {
    mOPERATOR.is_active = mOPERATOR.was_active_before_clutch;
    mOPERATOR.was_active_before_clutch = false;
  }
}

void stpTeleOperationCursor::RunAligningMTM(void) {
  // if clutched or align not needed, do nothing
  if (mOPERATOR.m_clutch.buttons[0] || !ConfigurationEvents.m_align_mtm.data) {
    return;
  }

  // set trajectory goal periodically??, this will track PSM motion
  //Orientate MTM with PSM
  stpMatrix mtmCartesianGoal;
  mtmCartesianGoal.SetTranslation(mMTM.m_setpoint_cp.GetMatrixTranslation());
  // Assign new rotation matrix
  stpMatrix mtmRotation;
  mtmRotation.SetRotation(m_registration_rotation.inverse() * mCURSOR.m_measured_cp.GetMatrixRotation());
  mtmCartesianGoal.FromNormalized(mtmRotation.GetMatrixRotation());
  // Send move command to mMTM
  mMTM.move_cp.publish(mtmCartesianGoal.transform_stamped_);

}

void stpTeleOperationCursor::TransitionAligningMTM(void) {
  // If the desired state is aligning MTM, just stay here
  if (!mTeleopState.DesiredStateIsNotCurrent()) {
    return;
  }

  // Check difference of orientation between mtm and cursor to enable
  stpMatrix desiredOrientation = UpdateAlignOffset();
  double orientationError = 0.0;
  if (ConfigurationEvents.m_align_mtm.data) {
    orientationError = desiredOrientation.AlignedWith(m_alignment_offset);
  }

  // If not active, use gripper and/or roll to detect if the user is ready
  if (!mOPERATOR.is_active) {
    // update gripper values
    const double gripper = mMTM.m_gripper_measured_js.position[0];
    if (gripper > mOPERATOR.gripper_max) {
      mOPERATOR.gripper_max = gripper;
    } else if (gripper < mOPERATOR.gripper_min) {
      mOPERATOR.gripper_min = gripper;
    }
    const double gripperRange = mOPERATOR.gripper_max - mOPERATOR.gripper_min;

    // checking rolling
    const double roll = acos(desiredOrientation.GetMatrixRotation().col(0).dot(mMTM.m_measured_cp.GetMatrixRotation().col(0)));
    if (roll > mOPERATOR.roll_max) {
      mOPERATOR.roll_max = roll;
    } else if (roll < mOPERATOR.roll_min) {
      mOPERATOR.roll_min = roll;
    }
    const double rollRange = mOPERATOR.roll_max - mOPERATOR.roll_min;

    if (gripperRange >= mOPERATOR.gripper_threshold) {
      mOPERATOR.is_active = true;
    } else if (rollRange >= mOPERATOR.roll_threshold) {
      mOPERATOR.is_active = true;
    } else if ((gripperRange + rollRange) > 0.8 * (mOPERATOR.gripper_threshold + mOPERATOR.roll_threshold)) {
      mOPERATOR.is_active = true;
    }
  }

  // finally check for transition
  if ((orientationError <= mOPERATOR.orientation_tolerance) && mOPERATOR.is_active) {
    if (mTeleopState.DesiredState() == "ENABLED") {
      mTeleopState.SetCurrentState("ENABLED");
    }
  } else {
    ROS_INFO("%s: unable to align MTM, angle is %f (deg).", this->mName.c_str(), orientationError);
  }
}

void stpTeleOperationCursor::EnterEnabled(void) {
  // Update MTM/CURSOR Previous position
  UpdateInitialState();

  // set MTM/CURSOR to Teleop (Cartesian Position Mode)
  geometry_msgs::WrenchStamped wrench;
  mMTM.servo_cf_body.publish(wrench);
  if (ConfigurationEvents.m_rotation_locked.data) {
    mMTM.lock_orientation.publish(mMTM.m_measured_cp.transform_stamped_.transform.rotation);
  } else {
    std_msgs::Empty empty;
    mMTM.unlock_orientation.publish(empty);
  }
  // Check if by any chance the clutch pedal was pressed
  if (mOPERATOR.m_clutch.buttons[0]) {
    Clutch(true);
  } else {
    set_following(true);
  }
}

void stpTeleOperationCursor::RunEnabled(void) {
  if (mMTM.m_measured_cp.isValid(mMTM.topicName.measured_cp)
    && mCURSOR.m_measured_cp.isValid(mCURSOR.topicName.measured_cp)) {
    // follow mode
    if (!mOPERATOR.m_clutch.buttons[0]){
      // Compute MTM Cartesian motion
      stpMatrix mtmPosition(mMTM.m_measured_cp);
      // translation
      stpMatrix mtmTransform;
      stpMatrix cursorTransform;
      if (ConfigurationEvents.m_translation_locked.data) {
        cursorTransform.SetTranslation(mCURSOR.CartesianInitial.GetMatrixTranslation());
      } else {
        mtmTransform.SetTranslation(mtmPosition.GetMatrixTranslation() - mMTM.CartesianInitial.GetMatrixTranslation());
        cursorTransform.SetTranslation(mtmTransform.GetMatrixTranslation() * ConfigurationEvents.m_scale.data);
        cursorTransform.SetTranslation(m_registration_rotation * cursorTransform.GetMatrixTranslation()
                                          + mCURSOR.CartesianInitial.GetMatrixTranslation());
      }
      // GetMatrixRotation
      if (ConfigurationEvents.m_rotation_locked.data) {
        cursorTransform.SetRotation(mCURSOR.CartesianInitial.GetMatrixRotation());
      } else {
        cursorTransform.SetRotation(m_registration_rotation * mtmTransform.GetMatrixRotation() * m_alignment_offset_initial);
      }

      // Compute desired cursor position
      stpMatrix cursorCartesianGoal;
      cursorCartesianGoal.SetTranslation(cursorTransform.GetMatrixTranslation());
      cursorCartesianGoal.SetRotation(cursorTransform.GetMatrixRotation());

      // take into account changes in CURSOR baseframe if any
      if (mBASEFRAME.m_measured_cp.isValid(mBASEFRAME.topicName.measured_cp)) {
        stpMatrix baseFrame(mBASEFRAME.m_measured_cp);
        stpMatrix baseFrameChange;
        baseFrameChange.SetRotation(baseFrame.Inverse()*mBASEFRAME.CartesianInitial.GetMatrixRotation());
        baseFrameChange.SetTranslation(baseFrame.Inverse()*mBASEFRAME.CartesianInitial.GetMatrixTranslation());
        // Update Cursor goal
        cursorCartesianGoal.SetRotation(baseFrameChange.GetMatrixRotation() * cursorCartesianGoal.GetMatrixRotation());
        cursorCartesianGoal.SetTranslation(baseFrameChange.GetMatrixRotation() * cursorCartesianGoal.GetMatrixTranslation());
        // Update alignmnet offset
        mtmTransform.FromNormalized(cursorCartesianGoal.GetMatrixRotation() * m_alignment_offset);
      }

      // Cursor goes to this cartesian position
      mCURSOR.servo_cp.publish(cursorCartesianGoal.transform_stamped_);
    }
  }
}

void stpTeleOperationCursor::TransitionEnabled(void) {
  if (mTeleopState.DesiredStateIsNotCurrent()) {
    set_following(false);
    mTeleopState.SetCurrentState(mTeleopState.DesiredState());
  }
}

void stpTeleOperationCursor::set_following(const bool following) {
  m_following = following;
}

void stpTeleOperationCursor::mtm_measured_cp_cb(const geometry_msgs::TransformStamped::ConstPtr &msg) {
  mMTM.m_measured_cp.GetROSMessage(*msg);
}

void stpTeleOperationCursor::mtm_setpoint_cp_cb(const geometry_msgs::TransformStamped::ConstPtr &msg) {
  mMTM.m_setpoint_cp.GetROSMessage(*msg);
}

void stpTeleOperationCursor::mtm_gripper_measured_js_cb(const sensor_msgs::JointState::ConstPtr &msg) {
  mMTM.m_gripper_measured_js = *msg;
}

void stpTeleOperationCursor::mtm_gripper_closed_cb(const std_msgs::Bool::ConstPtr &msg) {
  mMTM.m_gripper_closed = *msg;
}

void stpTeleOperationCursor::mtm_operating_state_cb(const crtk_msgs::operating_state::ConstPtr &msg) {
  mMTM.m_operating_state = *msg;
}

void stpTeleOperationCursor::cursor_measured_cp_cb(const geometry_msgs::TransformStamped::ConstPtr &msg) {
  mCURSOR.m_measured_cp.GetROSMessage(*msg);
}

void stpTeleOperationCursor::cursor_clicked_cb(const std_msgs::Bool::ConstPtr &msg) {
  mCURSOR.m_cursor_clicked = *msg;
}

void stpTeleOperationCursor::cursor_operating_state_cb(const crtk_msgs::operating_state::ConstPtr &msg) {
 mCURSOR.m_operating_state = *msg;
}

void stpTeleOperationCursor::operator_footpedal_cb(const sensor_msgs::Joy::ConstPtr &msg) {
  mOPERATOR.m_clutch = *msg;
}


















