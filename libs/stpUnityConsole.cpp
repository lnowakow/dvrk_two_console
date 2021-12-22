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
  std::string mtmrName = parser.GetStringValue(console_name, "MTMR", "name");
  std::string rCursorName = parser.GetStringValue(console_name, "MTMR", "CURSOR");
  Eigen::Isometry3d rBaseframe = parser.GetMatrixValue(console_name, "MTMR", "BASEFRAME");
  right_teleop.Init(default_config_file, mtmrName, rCursorName, rBaseframe);
  // Left Hand
  std::string mtmlName = parser.GetStringValue(console_name, "MTML", "name");
  std::string lCursorName = parser.GetStringValue(console_name, "MTML", "CURSOR");
  Eigen::Isometry3d lBaseframe = parser.GetMatrixValue(console_name, "MTML", "BASEFRAME");
  left_teleop.Init(default_config_file, mtmlName, lCursorName, lBaseframe);

  stp_console_events.topicName.read_teleop_cursor = console_name + "/teleop/read_teleop_cursor";

  stp_console_events.read_teleop_cursor = nh.subscribe(stp_console_events.topicName.read_teleop_cursor,
                                                       10,
                                                       &stpUnityConsole::stp_console_select_teleop_cursor_cb,
                                                       this);
  DominantHand("right");

  ROS_INFO("stpUnityConsole: %s successfully initialized!", console_name.c_str());
  mConfigured = true;
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

bool stpUnityConsole::ConfigureCursorTeleopJSON(const stpJsonParser &jsonTeleop) {
  return false;
}

void stpUnityConsole::teleop_enable(const bool &enable) {

}

void stpUnityConsole::cycle_teleop_psm_by_mtm(const std::string &mtmName) {

}

void stpUnityConsole::select_teleop_psm(const diagnostic_msgs::KeyValue mtmCURSOR) {
  // For readability
  const std::string mtmName = mtmCURSOR.key;
  const std::string cursorName = mtmCURSOR.value;

  // determine for which teleop pair this command is sent for
  stpTeleOperationCursor* selectedTeleop;
  if (right_teleop.getMTMName() == mtmName) {
    selectedTeleop = &right_teleop;
  } else if (left_teleop.getMTMName() == mtmName) {
    selectedTeleop = &left_teleop;
  } else {
    ROS_ERROR("%s: teleop key value \"%s\"is not a valid possibility", this->console_name.c_str(), mtmName.c_str());
    return;
  }

  // if the cursor string is empty, disable teleop for the given mtm
  if (cursorName == "") {
    selectedTeleop->state_command(std::string("disable"));
    ROS_INFO("%s: teleop %s has been unselected.", this->console_name.c_str(), selectedTeleop->getTeleopName().c_str());
    return;
  }

  // actual teleop to select
  // do a redundant check that the desired mtm's teleop is unselected
  diagnostic_msgs::KeyValue emptyMessage;
  emptyMessage.key = mtmName;
  emptyMessage.value = "";
  select_teleop_psm(emptyMessage);

  if (mTeleopCURSORRunning) {
    selectedTeleop->state_command(std::string("enable"));
  } else {
    selectedTeleop->state_command(std::string("align_mtm"));
  }
  ROS_INFO("%s: teleop %s has been selected.", this->console_name.c_str(), selectedTeleop->getTeleopName().c_str());

}

bool stpUnityConsole::GetCursorSelectedForMTM(const std::string &mtmName, std::string &psmName) const {
  return false;
}

bool stpUnityConsole::GetMTMSelectedForCursor(const std::string &cursorName, std::string &mtmName) const {
  return false;
}

void stpUnityConsole::EventSelectedTeleopTeleopCursors(void) const {

}

void stpUnityConsole::UpdateTeleopState(void) {
  // Check if teleop is enabled
  if (!mTeleopEnabled) {
    bool freezeNeeded = false;
  }
}

void stpUnityConsole::stp_console_select_teleop_cursor_cb(const diagnostic_msgs::KeyValueConstPtr& msg) {
  stp_console_events.m_read_teleop_cursor = *msg;
  select_teleop_psm(*msg);
}