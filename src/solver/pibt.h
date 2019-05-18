#pragma once

#include "solver.h"



class PIBT : public Solver {
protected:
  std::vector<float> epsilon;  // tie-breaker
  std::vector<int> eta;  // usually increment every step
  std::vector<float> priority;  // eta + epsilon

  void init();
  void allocate();

  void updatePriority();
  std::vector<Node*> createCandidates(Agent* a, std::vector<Node*> CLOSE_NODE);
  std::vector<Node*> createCandidates(Agent* a,
                                      std::vector<Node*> CLOSE_NODE,
                                      Node* tmp);
  bool priorityInheritance(Agent* a,
                           std::vector<Node*>& CLOSE_NODE,
                           std::vector<Agent*>& OPEN_AGENT,
                           std::vector<float>& PL);
  bool priorityInheritance(Agent* a,
                           Agent* aFrom,
                           std::vector<Node*>& CLOSE_NODE,
                           std::vector<Agent*>& OPEN_AGENT,
                           std::vector<float>& PL);
  bool priorityInheritance(Agent* a,
                           std::vector<Node*> C,
                           std::vector<Node*>& CLOSE_NODE,
                           std::vector<Agent*>& OPEN_AGENT,
                           std::vector<float>& PL);
  Node* chooseNode(Agent* a, std::vector<Node*> C);
  void updateC(std::vector<Node*>& C, Node* target, std::vector<Node*> CLOSE_NODE);

  float getDensity(Agent* a);  // density can be used as effective prioritization

public:
  PIBT(Problem* _P);
  PIBT(Problem* _P, std::mt19937* _MT);
  ~PIBT();

  bool solve();
  void update();

  std::string logStr();
};
