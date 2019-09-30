#pragma once

#include "solver.h"



class PIBT : public Solver {
protected:
  std::vector<float> epsilon;  // tie-breaker
  std::vector<int> eta;  // usually increment every step
  std::vector<float> priority;  // eta + epsilon

  void init();
  void allocate();

  virtual void updatePriority();
  Nodes createCandidates(Agent* a, Nodes CLOSE_NODE);
  Nodes createCandidates(Agent* a, Nodes CLOSE_NODE, Node* tmp);
  bool priorityInheritance(Agent* a,
                           Nodes& CLOSE_NODE,
                           Agents& OPEN_AGENT,
                           std::vector<float>& PL);
  bool priorityInheritance(Agent* a,
                           Agent* aFrom,
                           Nodes& CLOSE_NODE,
                           Agents& OPEN_AGENT,
                           std::vector<float>& PL);
  virtual bool priorityInheritance(Agent* a,
                                   Nodes C,
                                   Nodes& CLOSE_NODE,
                                   Agents& OPEN_AGENT,
                                   std::vector<float>& PL);
  virtual Node* chooseNode(Agent* a, Nodes C);
  void updateC(Nodes& C, Node* target, Nodes CLOSE_NODE);

  float getDensity(Agent* a);  // density can be used as effective prioritization

public:
  PIBT(Problem* _P);
  PIBT(Problem* _P, std::mt19937* _MT);
  ~PIBT();

  bool solve();
  virtual void update();

  virtual std::string logStr();
};
