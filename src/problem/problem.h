#pragma once
#include "../graph/graph.h"
#include "../agent/agent.h"
#include "../task/task.h"


class Problem {
protected:
  int timestep;
  Graph *G;
  std::vector<Agent*> A;
  std::vector<Task*> T_OPEN;   // open tasks
  std::vector<Task*> T_CLOSE;  // close tasks

  std::mt19937* MT;

  void init();

public:
  Problem(Graph* _G, std::vector<Agent*> _A, std::vector<Task*> _T);
  Problem(Graph* _G, std::vector<Agent*> _A);
  Problem(Graph* _G, std::vector<Agent*> _A, std::vector<Task*> _T, std::mt19937* _MT);
  Problem(Graph* _G, std::vector<Agent*> _A, std::mt19937* _MT);

  virtual ~Problem();

  virtual bool isSolved() { return false; }
  virtual void update() {}
  virtual bool allocated() { return false; }  // whether tasks are assigned beforehand?

  Graph* getG() { return G; }  // graph
  std::vector<Agent*> getA() { return A; }  // agents
  std::vector<Task*> getT() { return T_OPEN; }  // tasks (open)
  int getTerminationTime() { return timestep; }
  int getTimestep() { return timestep; }

  void assign(Task* tau);
  virtual void setAutoAssignement(bool flg) {}

  // for visualization
  bool visual_showicon;
  std::string visual_icon;

  virtual std::string logStr();
};
