#pragma once

#include "solver.h"


class TP : public Solver {
private:
  int timestep;
  Nodes endpoints;
  Paths paths;
  bool status;

  void init();
  void update();

  bool shouldAvoid(Agent* a, int timestep);
  std::vector<Task*> getExecutableTask(Agent* a, int timestep);
  void updatePath1(Agent* a, int startTime);
  void updatePath2(Agent* a, int startTime);
  Task* getNearestTask(Agent* a, std::vector<Task*>& tasks);

  Nodes getPath(Agent* a, int startTime);
  Nodes getPath(Agent* a,
                Node* s,
                Node* g,
                int startTime,
                bool futureCollision);
  Nodes getPickDelivPath(Agent* a,
                         Node* s,
                         Node* g1,
                         Node* g2,
                         int startTime,
                         bool futureCollision);

public:
  TP(Problem* _P, Nodes _ep);
  TP(Problem* _P, std::vector<int> _ep);
  ~TP();

  bool solve();

  std::string logStr();
};
