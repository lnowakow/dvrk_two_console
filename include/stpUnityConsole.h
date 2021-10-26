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
  class TeleopCursor {
   public:
    TeleopCursor(const std::string& name,
                const std::string& nameMTM,
                const std::string& nameCursor);

    /*! Create and Configure the robot arm. */
    void ConfigureTeleop(const stpJsonParser& jsonConfig);

    /*! Accessors */
    const std::string& Name(void) const;

    /*! Turn on/off selected */
    inline const bool & Selected(void) const {
      return mSelected;
    }
    inline void SetSelected(const bool selected) {
      mSelected = selected;
    }

   protected:
    bool mSelected;
    std::string m_name;
    std::string mMTMName;
    std::string mCURSORName;
  };

  stpUnityConsole();
  inline ~stpUnityConsole() {}

  /*! Configure console using JSON file. To test if the configuration successed, use method Configured() */
  void Configure(const std::string& filename);
  const bool & Configured(void) const;

  void Startup(void);
  void Run(void);
  void Cleanup(void);

  bool Connect(void);

 protected:
  bool mConfigured;
  bool mTeleopEnabled;
  bool mTeleopCursorRunning;
  bool mTeleopCursorAligning;

  /*! List to manage multiple PSM teleoperations */
  typedef std::map<std::string, TeleopCursor* > TeleopCursorList;
  TeleopCursorList mTeleopsCursor;

  /*! List to manage the teleopCursor components for each MTM */
  typedef std::multimap<std::string, TeleopCursor* > TeleopCursorByMTMList;
  typedef TeleopCursorByMTMList::iterator TeleopCursorByMTMIterator;
  typedef TeleopCursorByMTMList::const_iterator TeleopCursorByMTMConstIterator;
  TeleopCursorByMTMList mTeleopsCursorByMTM;
  /*! Name of default MTM to cycle teleops if no name is provided */
  std::string mTeleopMTMToCycle;

  bool AddTeleopCursorInterfaces(TeleopCursor* teleop);
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
