//
// Created by dvrk-1804 on 2021-09-20.
//

#ifndef DVRK_TWO_CONSOLE_INCLUDE_STPSTATEMACHINE_H_
#define DVRK_TWO_CONSOLE_INCLUDE_STPSTATEMACHINE_H_

#include "stpCallableVoidMethod.h"

#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

#define SECOND_DEFINITION true

class stpStateMachine {
 public:
  using StateType = std::string;

  stpStateMachine() { ROS_INFO("State Machine default constructor called"); };
  stpStateMachine(const std::string& name, const StateType initialState):
  mName(name),
  mFirstRun(true),
  mDesiredStateIsNotCurrent(false),
  mRunCallback(nullptr),
  mStateChangeCallback(nullptr),
  mCurrentState(initialState),
  mDesiredState(initialState),
  mPreviousState(initialState),
  mPreviousDesiredState(initialState)
  {
    ROS_INFO("%s: State machine successfully initialized.", mName.c_str());
    ROS_INFO("%s: State machine mFirstRun: %s", mName.c_str(), mFirstRun ? "true" : "false");
    AddState(initialState);
  }

  void AddState(const StateType state);
  void AddStates(const std::vector<StateType>& states);
  bool StateExists(const StateType state) const;

  /*! Add an allowed desired state.  One can only use
      SetDesiredState with allowed states. */
  void AddAllowedDesiredState(const StateType allowedState);

  /*! Set the Run callback for a given state. */
  //@{
  inline void SetRunCallback(const StateType state, stpCallableVoidBase * callback) {
    if (!StateExists(state)) {
      ROS_INFO("mtsStateMachine::SetRunCallback: %s , state [%s] doesn't exist.  Use AddState first.", mName.c_str(), state.c_str());
    }
    mRunCallbacks[state] = callback;
  }
#if SECOND_DEFINITION
  template <class __classType>
      inline void SetRunCallback(const StateType state,
                                 void (__classType::*method)(void),
                                 __classType * classInstantiation) {
    this->SetRunCallback(state, new stpCallableVoidMethod<__classType>(method, classInstantiation));
  }
  //@}
#endif

  /*! Set the Run callback called for all states, this method is
    called before the state specific Run callback. */
  //@{
  inline void SetRunCallback(stpCallableVoidBase * callback) {
    mRunCallback = callback;
  }
#if SECOND_DEFINITION
  template <class __classType>
      inline void SetRunCallback(void (__classType::*method)(void),
                                 __classType * classInstantiation) {
    this->SetRunCallback(new stpCallableVoidMethod<__classType>(method, classInstantiation));
  }
  //@}
#endif

  /*! Set the Enter callback for a given state.  This method is
    called only once, before the Run callback. */
  //@{
  inline void SetEnterCallback(const StateType state, stpCallableVoidBase * callback) {
    if (!StateExists(state)) {
      ROS_INFO("mtsStateMachine::SetEnterCallback: %s, state [%s] doesn't exist.  Use AddState first.", mName.c_str(), state.c_str());
    }
    mEnterCallbacks[state] = callback;
  }
#if SECOND_DEFINITION
  template <class __classType>
      inline void SetEnterCallback(const StateType state,
                                   void (__classType::*method)(void),
                                   __classType * classInstantiation) {
    this->SetEnterCallback(state, new stpCallableVoidMethod<__classType>(method, classInstantiation));
  }
  //@}
#endif

  /*! Set the Leave callback for a given state.  Called once when
    leaving the current state. */
  //@{
  inline void SetLeaveCallback(const StateType state, stpCallableVoidBase * callback) {
    if (!StateExists(state)) {
      ROS_INFO("mtsStateMachine::SetLeaveCallback: %s, state [%s] doesn't exist.  Use AddState first.", mName.c_str(), state.c_str());
    }
    mLeaveCallbacks[state] = callback;
  }
#if SECOND_DEFINITION
  template <class __classType>
      inline void SetLeaveCallback(const StateType state,
                                   void (__classType::*method)(void),
                                   __classType * classInstantiation) {
    this->SetLeaveCallback(state, new stpCallableVoidMethod<__classType>(method, classInstantiation));
  }
  //@}
#endif

  /*! Set the Transition callback for a given state.  This callback
    is called after the Run callback for the current state. */
  //@{
  inline void SetTransitionCallback(const StateType state, stpCallableVoidBase * callback) {
    if (!StateExists(state)) {
      ROS_INFO("mtsStateMachine::SetTransitionCallback: %s, state [%s] doesn't exist.  Use AddState first.", mName.c_str(), state.c_str());
    }
    mTransitionCallbacks[state] = callback;
  }
#if SECOND_DEFINITION
  template <class __classType>
      inline void SetTransitionCallback(const StateType state,
                                        void (__classType::*method)(void),
                                        __classType * classInstantiation) {
    this->SetTransitionCallback(state, new stpCallableVoidMethod<__classType>(method, classInstantiation));
  }
  //@}
#endif

  /*! Set the state changed callback. */
  //@{
  inline void SetStateChangedCallback(stpCallableVoidBase * callback) {
    mStateChangeCallback = callback;
  }
#if SECOND_DEFINITION
  template <class __classType>
      inline void SetStateChangedCallback(void (__classType::*method)(void), __classType* classInstantiation) {
    this->SetStateChangedCallback(new stpCallableVoidMethod<__classType>(method, classInstantiation));
  }
  //@}
#endif

  void Run(void);

  inline const StateType & CurrentState(void) const {
    return mCurrentState;
  }

  inline const StateType & DesiredState(void) const {
    return mDesiredState;
  }

  inline const StateType & PreviousState(void) const {
    return mPreviousState;
  }

  inline const StateType & PreviousDesiredState(void) const {
    return mPreviousDesiredState;
  }

  /*! Set the desired state.  This will check if the state is a
    possible desired state. */
  void SetDesiredState(const StateType & desiredState);

  /*! Set the current state.  This will check if the state is a
    valid state.  Leave and enter callbacks will also be called.
    Finally all callback pointers for the current state (run and
    transition) will be updated to avoid a callback lookup by state
    name in the Run method. */
  void SetCurrentState(const StateType & newState);

  /*! Check if the desired and current states are different.  This
      allows to avoid a string compare to determine if a transition
      is desired. */
  inline bool DesiredStateIsNotCurrent(void) const {
    return mDesiredStateIsNotCurrent;
  }

 protected:


  void UpdateCurrentCallbacks(void);

  std::string mName;
  bool mFirstRun;
  bool mDesiredStateIsNotCurrent;

  typedef std::map<StateType, stpCallableVoidBase *> CallbackMap;
  CallbackMap mEnterCallbacks,
              mRunCallbacks,
              mLeaveCallbacks,
              mTransitionCallbacks;
  stpCallableVoidBase *mRunCallback,
                      *mCurrentRunCallback{},
                      *mCurrentTransitionCallback{},
                      *mStateChangeCallback;

  StateType mCurrentState,
            mDesiredState,
            mPreviousState,
            mPreviousDesiredState;

  typedef std::map<StateType, bool> StateMap;
  StateMap mStates;

};

#endif //DVRK_TWO_CONSOLE_INCLUDE_STPSTATEMACHINE_H_
