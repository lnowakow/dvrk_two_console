//
// Created by dvrk-1804 on 2021-10-26.
//

#ifndef DVRK_TWO_CONSOLE_LIBS_STPUNITYCONSOLE_H_
#define DVRK_TWO_CONSOLE_LIBS_STPUNITYCONSOLE_H_

#include "stpJsonParser.h"
#include "stpTeleOperationCursor.h"
#include <string>
#include <map>

#include "stpTeleOperationCursor.h"

#include <diagnostic_msgs/KeyValue.h>

class stpUnityConsole {
 public:
  stpUnityConsole();
  stpUnityConsole(const std::string &consoleName,
                  const std::string &filename,
                  const std::string &rightTeleopName,
                  const std::string &leftTeleopName);
  inline ~stpUnityConsole() { delete cursor_teleop; };

  /*! Configure console using JSON file. To test if the configuration successed, use method Configured() */
  void Configure(const std::string& filename);

  /*! Method to check if the configuration was successful, ideally called after a call to Configure */
  const bool & Configured(void) const;

  /*! Set whether the user wants right or left handed control */
  void DominantHand(const std::string &hand);

  void Startup(void);
  void Run(void);
  void Cleanup(void);

  bool Connect(void);

 protected:
  bool mConfigured;
  bool mTeleopEnabled;
  bool mTeleopDesired;
  bool mTeleopCURSORRunning;
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
    struct {
      std::string select_teleop_psm;
    } topicName;
    stpWrite select_teleop_psm;

    diagnostic_msgs::KeyValue m_select_teleop_psm;
  } dVRK_console_events;

  struct {
    struct {
      std::string read_teleop_cursor;
      std::string teleop_enabled;
    } topicName;
    stpRead read_teleop_cursor;
    stpWrite teleop_enabled;

    diagnostic_msgs::KeyValue m_read_teleop_cursor;
    bool m_teleop_enabled;
  } stp_console_events;

  struct {
    struct {
      std::string teleop_psm_selected;
      std::string teleop_psm_unselected;
    } topicName;
    stpWrite teleop_psm_selected;
    stpWrite teleop_psm_unselected;
  } dVRKConfigurationEvents;

 private:

  ros::NodeHandle nh;

  stpJsonParser parser;
  std::string console_name;
  stpTeleOperationCursor right_teleop, left_teleop;
  stpTeleOperationCursor* cursor_teleop = nullptr;

  void stp_console_select_teleop_cursor_cb(const diagnostic_msgs::KeyValueConstPtr& msg);

};

#endif //DVRK_TWO_CONSOLE_LIBS_STPUNITYCONSOLE_H_
