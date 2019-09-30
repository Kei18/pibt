#pragma once
#include "problem.h"

// naive Iterative MAPF
class IMAPF_FAIR : public Problem {
private:
  int taskLimit;
  int taskSum;
  std::vector<int> goalCounts;
  void init();
  void allocate(Agent* a);

public:
  IMAPF_FAIR(Graph* _G, Agents _A, int _taskLimit);
  IMAPF_FAIR(Graph* _G, Agents _A, int _taskLimit, std::mt19937* _MT);
  ~IMAPF_FAIR();

  bool isSolved();
  void update();

  std::string logStr();
};
