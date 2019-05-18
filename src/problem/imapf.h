#pragma once
#include "problem.h"

// naive Iterative MAPF
class IMAPF : public Problem {
private:
  int taskSum;
  const int taskLimit;
  Nodes nodes;

  std::string taskSumStr;

  void init();
  void allocate(Agent* a);

public:
  IMAPF(Graph* _G, Agents _A, int _taskLimit);
  IMAPF(Graph* _G, Agents _A, int _taskLimit, std::mt19937* _MT);
  ~IMAPF();

  bool isSolved();
  void update();

  std::string logStr();
};
