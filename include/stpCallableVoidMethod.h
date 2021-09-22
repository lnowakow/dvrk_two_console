//
// Created by dvrk-1804 on 2021-09-21.
//

#ifndef DVRK_TWO_CONSOLE_LIBS_STPCALLABLECLASS_H_
#define DVRK_TWO_CONSOLE_LIBS_STPCALLABLECLASS_H_

#include "stpCallableVoidBase.h"

template<class _classType>
class stpCallableVoidMethod : public stpCallableVoidBase {
  typedef stpCallableVoidBase BaseType;
  typedef void (_classType::*ActionType)(void);

 protected:
  ActionType Action;
  _classType * ClassInstantiation;

 public:
  stpCallableVoidMethod(void): BaseType(), ClassInstantiation(0) {}
  stpCallableVoidMethod(ActionType action, _classType * classInstantiation):
    BaseType(),
    Action(action),
    ClassInstantiation(classInstantiation)
  {}

  inline void Execute(void) {
    (ClassInstantiation->*Action)();
  }

  ~stpCallableVoidMethod() {}

};

#endif //DVRK_TWO_CONSOLE_LIBS_STPCALLABLECLASS_H_
