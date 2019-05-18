#pragma once

#include "solver.h"

struct Conflict {
  Agent* a;  // this agent
  int t;     // at timestep t
  Node* v;   // cannot use v
};

using Constraint = std::vector<Conflict*>;
using Constraints = std::vector<Constraint>;
using Block = std::vector<Agent*>;

struct CTNode {  // conflict-tree node
  Constraint c;  // conflicts
  Paths paths;   // current path
  int cost;      // sum of paths
  CTNode* p;     // parent
  bool valid;

  std::vector<int> fmins;  // for ECBS
  int LB;  // for ECBS
};


class CBS : public Solver {
protected:
  bool ID;  // independent detection

  void init();
  virtual void invoke(CTNode* node, Block& block);
  Constraints valid(CTNode* node, Block& block);
  void calcCost(CTNode* node, Block& block);
  void formalizePathBlock(CTNode* node, Block& block);
  virtual std::vector<Node*> AstarSearch(Agent* a, CTNode* node);
  Constraint getConstraitnsForAgent(CTNode* ctNode, Agent* a);
  Constraint getConstraitns(CTNode* ctNode);
  bool checkBlocks(Paths& paths, std::vector<Block>& whole);
  virtual bool solvePart(Paths& paths, Block& block);
  bool isDuplicatedConflict(Constraint &constraint, Conflict* c);
  int countCollisions(CTNode* c, Paths& paths);

public:
  CBS(Problem* _P);
  CBS(Problem* _P, bool _ID);
  ~CBS();

  bool solve();

  virtual std::string logStr();
};
