//
// Created by dvrk-1804 on 2021-10-26.
//

#include "../include/stpUnityConsole.h"

stpUnityConsole::stpUnityConsole() {

}

stpUnityConsole::stpUnityConsole(const std::string &consoleName,
                                 const std::string &filename,
                                 const std::string &rightTeleopName,
                                 const std::string &leftTeleopName) :
                                 right_teleop(rightTeleopName),
                                 left_teleop(leftTeleopName)
                                 {
  console_name = consoleName;
  ROS_INFO("Console name: %s", console_name.c_str());
  Configure(filename);
}

void stpUnityConsole::Configure(const std::string &filename) {

  mConfigured = false;
  parser.openFile(filename.c_str());
  std::string default_config_file = parser.GetStringValue("Topic-Config");

  // Console 1 Teleoperation configuration
  // Right Hand
  ConfigureCursorTeleopJSON("MTMR", default_config_file);
//  std::string mtmrName = parser.GetStringValue(console_name, "MTMR", "name");
//  std::string rCursorName = parser.GetStringValue(console_name, "MTMR", "CURSOR");
//  Eigen::Isometry3d rBaseframe = parser.GetMatrixValue(console_name, "MTMR", "BASEFRAME");
//  right_teleop.Init(default_config_file, mtmrName, rCursorName, rBaseframe);
  // Left Hand
  ConfigureCursorTeleopJSON("MTML", default_config_file);
//  std::string mtmlName = parser.GetStringValue(console_name, "MTML", "name");
//  std::string lCursorName = parser.GetStringValue(console_name, "MTML", "CURSOR");
//  Eigen::Isometry3d lBaseframe = parser.GetMatrixValue(console_name, "MTML", "BASEFRAME");
//  left_teleop.Init(default_config_file, mtmlName, lCursorName, lBaseframe);

  // stpConsoleEvents
  stp_console_events.topicName.read_teleop_cursor = console_name + "/teleop/read_teleop_cursor";
  stp_console_events.topicName.teleop_cursor_selected = console_name + "/teleop/teleop_cursor_selected";
  stp_console_events.topicName.teleop_cursor_unselected = console_name + "/teleop/teleop_cursor_unselected";
  stp_console_events.topicName.teleop_enabled = console_name + "/teleop/teleop_enabled";

  stp_console_events.read_teleop_cursor = nh.subscribe(stp_console_events.topicName.read_teleop_cursor,
                                                       10,
                                                       &stpUnityConsole::stp_console_select_teleop_cursor_cb,
                                                       this);
  stp_console_events.teleop_cursor_selected = nh.advertise<diagnostic_msgs::KeyValue>(stp_console_events.topicName.teleop_cursor_selected, 10);
  stp_console_events.teleop_cursor_unselected = nh.advertise<diagnostic_msgs::KeyValue>(stp_console_events.topicName.teleop_cursor_unselected, 10);
  stp_console_events.teleop_enabled = nh.advertise<std_msgs::Bool>(stp_console_events.topicName.teleop_enabled, 10);
  // Choose Dominant Hand
  DominantHand("right");

  // dVRKConsoleEvents
  dVRK_console_events.topicName.operator_present = "/console/operator_present"; // Change JSON config formats.
  dVRK_console_events.operator_present = nh.subscribe(dVRK_console_events.topicName.operator_present,
                                                      10,
                                                      &stpUnityConsole::dvrk_console_events_operator_present_cb,
                                                      this);




  ROS_INFO("stpUnityConsole: %s successfully initialized!", console_name.c_str());
  mConfigured = true;
}

bool stpUnityConsole::ConfigureCursorTeleopJSON(const std::string &mtm_Name, const std::string &default_config_file) {

  // Right Hand
  std::string mtmName = parser.GetStringValue(console_name, mtm_Name, "name");
  if (mtmName == "") {
    ROS_ERROR("%s: ConfigureCursorTeleopJSON: Invalid MTM name given for teleop", this->console_name.c_str());
  }
  std::string cursorName = parser.GetStringValue(console_name, mtm_Name, "CURSOR");
  if (cursorName == "") {
    ROS_ERROR("%s: ConfigureCursorTeleopJSON: Invalid Cursor name given for teleop", this->console_name.c_str());
  }

  Eigen::Isometry3d Baseframe = parser.GetMatrixValue(console_name, mtm_Name, "BASEFRAME");
  if (Baseframe.matrix().isZero()) {
    ROS_ERROR("%s: ConfigureCursorTeleopJSON: %s Baseframe is empty", this->console_name.c_str(), mtmName.c_str());
  }
  // check if pair already exists and then add it
  const std::string name = mtmName + "-" + cursorName;
  const auto teleopIterator = mTeleopsCursor.find(name);
  stpTeleOperationCursor *teleopPointer = nullptr;
  if (teleopIterator == mTeleopsCursor.end()) {
    // create a new teleop
    teleopPointer = new stpTeleOperationCursor();
    teleopPointer->Init(default_config_file, mtmName, cursorName, Baseframe);
    teleopPointer->SetSelected(false);
    mTeleopsCursor[name] = teleopPointer;
  } else {
    ROS_ERROR("%s: ConfigureCursorTeleopJSON: There is already a teleop for the pair %s.", this->console_name.c_str(), mtmName.c_str());
    return false;
  }

  return true;
}

const bool & stpUnityConsole::Configured(void) const {
    return mConfigured;
}

void stpUnityConsole::DominantHand(const std::string &hand) {
  if (hand == "right") {
    cursor_teleop = &right_teleop;
  } else if (hand == "left") {
    cursor_teleop = &left_teleop;
  } else {
    ROS_ERROR("%s: Invalid dominant hand requested: %s. Options are only \"right\" or \"left\"", this->console_name.c_str(), hand.c_str());
    return;
  }
  ROS_INFO("%s: Dominant hand selected for Unity cursor functions: %s hand", this->console_name.c_str(), hand.c_str());

}

void stpUnityConsole::Startup(void) {
}

void stpUnityConsole::Run(void) {
  try {
    ros::spinOnce();
    cursor_teleop->Run();
  } catch (std::exception& e) {
    ROS_ERROR("%s: Failed with %s", console_name.c_str(), e.what());
  }
}

void stpUnityConsole::Cleanup(void) {
}

bool stpUnityConsole::Connect(void) {
  return false;
}

void stpUnityConsole::teleop_enable(const bool &enable) {
  mTeleopEnabled = enable;
  mTeleopDesired = enable;
  stp_console_events.teleop_enabled.publish(mTeleopEnabled);
  UpdateTeleopState();
}

void stpUnityConsole::cycle_teleop_psm_by_mtm(const std::string &mtmName) {

}

void stpUnityConsole::select_teleop_cursor(const diagnostic_msgs::KeyValue mtmCURSOR) {

  // For readability
  const std::string mtmName = mtmCURSOR.key;
  const std::string cursorName = mtmCURSOR.value;


  // if the cursor string is empty, disable teleop for the given mtm
  if (cursorName == "") {
    auto range = mTeleopsCursor.equal_range(mtmName);
    for (auto iter = range.first; iter != range.second; ++iter) {
      // look for the teleop that was selected if any
      if (iter->second->Selected()) {
        iter->second->SetSelected(false);
        // if teleop Cursor is active, enable/disable components now
        if (mTeleopEnabled) {
          iter->second->state_command(std::string("disabled"));
        }
        ROS_INFO("%s: teleop \"%s\" has been unselected. ", this->console_name.c_str(), iter->second->getTeleopName().c_str());
      }
    }
    EventSelectedTeleopCursors();
    return;
  }

  // actual teleop to select
  std::string name = mtmName + "-" + cursorName;
  const auto teleopIterator = mTeleopsCursor.find(name);
  if (teleopIterator == mTeleopsCursor.end()) {
    ROS_ERROR("%s: unable to select \"%s\", this component does not exist.", this->console_name.c_str(), name.c_str());
    EventSelectedTeleopCursors();
    return;
  }
  // do a redundant check that the desired mtm's teleop is unselected
  diagnostic_msgs::KeyValue emptyMessage;
  emptyMessage.key = mtmName;
  emptyMessage.value = "";
  select_teleop_cursor(emptyMessage);
  // now turn on teleop
  teleopIterator->second->SetSelected(true);
  if (mTeleopEnabled) {
    if (mTeleopCursorRunning) {
      teleopIterator->second->state_command(std::string("enable"));
    } else {
      teleopIterator->second->state_command(std::string("align_mtm"));
    }
  }
  ROS_INFO("%s: teleop %s has been selected.", this->console_name.c_str(), teleopIterator->second->getTeleopName().c_str());
  EventSelectedTeleopCursors();
}

bool stpUnityConsole::GetCursorSelectedForMTM(const std::string &mtmName, std::string &psmName) const {
  return false;
}

bool stpUnityConsole::GetMTMSelectedForCursor(const std::string &cursorName, std::string &mtmName) const {
  return false;
}

void stpUnityConsole::EventSelectedTeleopCursors(void) const {
  for (auto &iter : mTeleopsCursor) {
    if (iter.second->Selected()) {
      diagnostic_msgs::KeyValue message;
      message.key = iter.second->getMTMName();
      message.value = iter.second->getCURSORName();
      stp_console_events.teleop_cursor_selected.publish(message);
    } else {
      diagnostic_msgs::KeyValue message;
      message.key = iter.second->getMTMName();
      message.value = iter.second->getCURSORName();
      stp_console_events.teleop_cursor_unselected.publish(message);
    }
  }
}

void stpUnityConsole::UpdateTeleopState(void) {
  // Check if teleop is enabled
  if (!mTeleopEnabled) {
    bool freezeNeeded = false;
    for (auto & iterTeleopCursor : mTeleopsCursor) {
      iterTeleopCursor.second->state_command(std::string("disable"));
      if (mTeleopCursorRunning) {
        freezeNeeded = true;
      }
      mTeleopCursorRunning = false;
    }
    if (freezeNeeded) {
      for (auto & iterTeleopCursor : mTeleopsCursor) {
        iterTeleopCursor.second->Freeze();
      }
    }
    return;
  }

  // if none are running, freeze
  if (!mTeleopCursorRunning) {
    for (auto & iterTeleopCursor : mTeleopsCursor) {
      iterTeleopCursor.second->Freeze();
    }
  }
   // all fine
  bool readyForTeleop = mOperatorPresent;

  // Not considering SUJ clutches for this
  // Check if operator is present
  if (!readyForTeleop) {
    // keep MTMs aligned
    for (auto & iterTeleopCursor : mTeleopsCursor) {
      if (iterTeleopCursor.second->Selected()) {
        iterTeleopCursor.second->state_command(std::string("align_mtm"));
      } else {
        iterTeleopCursor.second->state_command(std::string("disable"));
      }
    }
    mTeleopCursorRunning = false;
    return;
  }

}

void stpUnityConsole::stp_console_select_teleop_cursor_cb(const diagnostic_msgs::KeyValueConstPtr& msg) {
  stp_console_events.m_read_teleop_cursor = *msg;
  select_teleop_cursor(*msg);
}

void stpUnityConsole::dvrk_console_events_operator_present_cb(const sensor_msgs::Joy::ConstPtr& msg) {
  if (msg->buttons[0] == 1) {
    mOperatorPresent = true;
  } else {
    mOperatorPresent = false;
  }
}
