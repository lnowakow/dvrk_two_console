//
// Created by dvrk-1804 on 2021-09-20.
//

#include "../include/stpStateMachine.h"

void stpStateMachine::AddState(const StateType state) {
  if (StateExists(state)) {
    ROS_INFO("stpStateMachine::AddState: %s, state %s already exists", mName.c_str(), state.c_str());
  }
  mStates[state] = false;
}

void stpStateMachine::AddStates(const std::vector<stpStateMachine::StateType>& states) {
  typedef std::vector<StateType> VectorType;
  const VectorType::const_iterator end = states.end();
  VectorType::const_iterator iter = states.begin();
  for (; iter != end; ++iter) {
    AddState(*iter);
  }
}

bool stpStateMachine::StateExists(const stpStateMachine::StateType state) const {
  const StateMap::const_iterator found = mStates.find(state);
  return (found != mStates.end());
}

void stpStateMachine::AddAllowedDesiredState(const stpStateMachine::StateType allowedState) {
  if (StateExists(allowedState)) {
    mStates[allowedState] = true;
  } else {
    ROS_INFO("stpStateMachine::AddAllowedDesiredStates: %s, state %s needs to be added first.", mName.c_str(), allowedState.c_str());
  }
}

void stpStateMachine::Run(void) {
  // On first run, call enter callback for initial state
  if (mFirstRun) {
    UpdateCurrentCallbacks();

    // find new state enter callback
    CallbackMap::iterator callback;
    callback = mEnterCallbacks.find(mCurrentState);
    if (callback != mEnterCallbacks.end()) {
      callback->second->Execute();
    }

    // User callback if provided
    if (mStateChangeCallback) {
      mStateChangeCallback->Execute();
    }
    mFirstRun = false;
  }
  // Check if a transition should happen
  if (mCurrentTransitionCallback) {
    mCurrentTransitionCallback->Execute();
  }

  // Run current state method
  if (mRunCallback) {
    mRunCallback->Execute();
  }
  if (mCurrentRunCallback) {
    mCurrentRunCallback->Execute();
  }
}

void stpStateMachine::SetDesiredState(const StateType & desiredState) {
  const StateMap::const_iterator state = mStates.find(desiredState);
  if ((state != mStates.end()) && (state->second)) {
    mPreviousDesiredState = mDesiredState;
    mDesiredState = desiredState;
    mDesiredStateIsNotCurrent = (mDesiredState != mCurrentState);
    return;
  }
  ROS_INFO("stpStateMachine::SetDesiredState: %s, doesnt exist or is not allowed as a desired state", desiredState.c_str());
}

void stpStateMachine::SetCurrentState(const stpStateMachine::StateType & newState) {
  // Check if the state exists
  const StateMap::const_iterator state = mStates.find(newState);
  if (state == mStates.end()) {
    ROS_INFO("mtsSTateMachine::SetCurrentState: %s, doesnt exist", newState.c_str());
    return;
  }

  CallbackMap::iterator callback;

  //find current state leave callback
  callback = mLeaveCallbacks.find(mCurrentState);
  if (callback != mLeaveCallbacks.end()) {
    callback->second->Execute();
  }
  // Set the new state and update current callbacks
  mPreviousState = mCurrentState;
  mCurrentState = newState;
  mDesiredStateIsNotCurrent = (mDesiredState != mCurrentState);

  // Find new state enter callback
  callback = mEnterCallbacks.find(mCurrentState);
  if (callback != mEnterCallbacks.end()) {
    callback->second->Execute();
  }

  // User callback if provided
  if (mStateChangeCallback) {
    mStateChangeCallback->Execute();
  }

  // Update current callbacks
  UpdateCurrentCallbacks();
}

void stpStateMachine::UpdateCurrentCallbacks(void) {
  CallbackMap::iterator callback;
  // Find state run callback
  callback = mRunCallbacks.find(mCurrentState);
  if (callback != mRunCallbacks.end()) {
    mCurrentRunCallback = callback->second;
  } else {
    mCurrentRunCallback = 0;
  }

  // Find transition callback
  callback = mTransitionCallbacks.find(mCurrentState);
  if (callback != mTransitionCallbacks.end()) {
    mCurrentTransitionCallback = callback->second;
  } else {
    mCurrentTransitionCallback = 0;
  }
}

























