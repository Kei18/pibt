#pragma once

#include "whca.h"


class HCA : public WHCA {
public:
  HCA(Problem* _P) : WHCA(_P) {};

  std::string logStr() {
    std::string str;
    str += "[solver] type:HCA\n";
    str += Solver::logStr();
    return str;
  }
};
