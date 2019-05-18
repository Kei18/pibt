#pragma once
#include "problem.h"


class MAPD : public Problem {
private:
  Nodes pickupNodes;
  Nodes deliveryNodes;
  int taskCnt;
  int taskNum;
  float taskFrequency;
  bool autoAssignment;

  void init();
  void autoAssign();
  void updateStatus();
  void createTask();

public:
  MAPD(Graph* _G,
       Agents _A,
       Nodes pickupNodes,
       Nodes deliveryNodes,
       int taskNum, float taskFrequency);
  MAPD(Graph* _G,
       Agents _A,
       std::vector<int> pickupNodes,
       std::vector<int> deliveryNodes,
       int taskNum, float taskFrequency);
  MAPD(Graph* _G,
       Agents _A,
       Nodes pickupNodes,
       Nodes deliveryNodes,
       int taskNum, float taskFrequency, std::mt19937* _MT);
  MAPD(Graph* _G,
       Agents _A,
       std::vector<int> pickupNodes,
       std::vector<int> deliveryNodes,
       int taskNum, float taskFrequency, std::mt19937* _MT);
  ~MAPD();
  bool isSolved();
  void update();
  void setAutoAssignement(bool flg) { autoAssignment = flg; }  // by solver
  bool allocated() { return false; }

  std::string logStr();
};
