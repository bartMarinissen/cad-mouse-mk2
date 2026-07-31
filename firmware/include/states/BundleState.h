#pragma once

#include "State.h"

class BundleState : public State {
 public:
  void enter() override;
  void update() override;
  void exit() override;
};
