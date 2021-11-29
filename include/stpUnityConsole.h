//
// Created by dvrk-1804 on 2021-10-26.
//

#ifndef DVRK_TWO_CONSOLE_LIBS_STPUNITYCONSOLE_H_
#define DVRK_TWO_CONSOLE_LIBS_STPUNITYCONSOLE_H_

#include <string>
#include <map>

#include "stpTeleOperationCursor.h"

#include <diagnostic_msgs/KeyValue.h>

class stpUnityConsole : public stpTeleOperationCursor {

  stpUnityConsole();
  inline ~stpUnityConsole() {}

  /*! Configure console using JSON file. To test if the configuration successed, use method Configured() */
  void Configure(const std::string& filename);

  void Startup(void);
  void Run(void);
  void Cleanup(void);

  bool Connect(void);

 protected:
  bool mConfigured;
  bool mTeleopEnabled;
  bool mTeleopCursorRunning;
  bool mTeleopCursorAligning;

  bool ConfigureCursorTeleopJSON(const stpJsonParser& jsonTeleop);

  void teleop_enable(const bool& enable);
  void cycle_teleop_psm_by_mtm(const std::string& mtmName);
  void select_teleop_psm(const diagnostic_msgs::KeyValue mtmPSM);
  bool GetCursorSelectedForMTM(const std::string& mtmName, std::string& psmName) const;
  bool GetMTMSelectedForCursor(const std::string& cursorName, std::string& mtmName) const;
  void EventSelectedTeleopTeleopCursors(void) const;
  void UpdateTeleopState(void);

  struct {
    stpWrite teleop_enabled;
  } console_events;

  struct {
    stpWrite teleop_psm_selected;
    stpWrite teleop_psm_unselected;
  } ConfigurationEvents;

};

#endif //DVRK_TWO_CONSOLE_LIBS_STPUNITYCONSOLE_H_
