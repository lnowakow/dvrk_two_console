//
// Created by dvrk-1804 on 2021-10-26.
//

#include "../include/stpUnityConsole.h"

stpUnityConsole::stpUnityConsole() {

}

void stpUnityConsole::Configure(const std::string &filename) {

  mConfigured = false;
  parser.openFile(filename.c_str());
  std::string default_config_file = parser.GetStringValue("Topic-Config");

  // Console 1 Teleoperation configuration
  std::string cursor1Name = parser.GetStringValue("Console1", "CURSOR");
  // Right Hand
  std::string mtm1rName = parser.GetStringValue("Console1", "MTMR", "name");
  Eigen::Isometry3d baseframe1r = parser.GetMatrixValue("Console1", "MTMR", "BASEFRAME");
  right_teleop.Init(default_config_file, mtm1rName, cursor1Name, baseframe1r);
  // Left Hand
  std::string mtm1lName = parser.GetStringValue("Console1", "MTMR", "name");
  Eigen::Isometry3d baseframe1l = parser.GetMatrixValue("Console1", "MTMR", "BASEFRAME");
  left_teleop.Init(default_config_file, mtm1lName, cursor1Name, baseframe1l);

  mConfigured = true;
}

const bool & stpUnityConsole::Configured(void) const {
    return mConfigured;
}
void stpUnityConsole::Startup(void) {
}

void stpUnityConsole::Run(void) {
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

void stpUnityConsole::select_teleop_psm(const diagnostic_msgs::KeyValue mtmPSM) {

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

