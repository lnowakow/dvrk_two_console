//
// Created by dvrk-1804 on 2021-10-26.
//

#include "../include/stpUnityConsole.h"

stpUnityConsole::stpUnityConsole() {

}

void stpUnityConsole::Configure(const std::string &filename) {

}

const bool &stpUnityConsole::Configured(void) const {

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

bool stpUnityConsole::AddTeleopCursorInterfaces(stpUnityConsole::TeleopCursor *teleop) {
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

}

