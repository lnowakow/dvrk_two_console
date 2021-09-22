//
// Created by dvrk-1804 on 2021-09-21.
//

#ifndef DVRK_TWO_CONSOLE_LIBS_STPCALLABLEVOIDBASE_H_
#define DVRK_TWO_CONSOLE_LIBS_STPCALLABLEVOIDBASE_H_

class stpCallableVoidBase {
 public:
  inline stpCallableVoidBase(void) {}
  inline virtual ~stpCallableVoidBase() {}

  virtual void Execute(void) = 0;

};

#endif //DVRK_TWO_CONSOLE_LIBS_STPCALLABLEVOIDBASE_H_
