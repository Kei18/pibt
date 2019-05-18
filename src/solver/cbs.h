#pragma once

#include "solver.h"

struct Conflict {
  Agent* a;  // this agent
  int t;     // at timestep t
  Node* v;   // cannot use v
  Node* u;   // prev
  bool onNode;  // true or false
  std::string key;
};

using Constraint = std::vector<Conflict*>;
using Constraints = std::vector<Constraint>;

struct CTNode {  // conflict-tree node
  Constraint c;  // conflicts
  Paths paths;   // current path
  int cost;      // sum of paths
  CTNode* p;     // parent
  bool valid;

  std::vector<int> fmins;  // for ecbs
  int LB;
};


class CBS : public Solver {
protected:
  bool ID;  // independent detection
  std::unordered_map<std::string, Nodes> knownPaths;

  void init();
  virtual void invoke(CTNode* node, Agents& block);
  Constraints valid(CTNode* node, Agents& block);
  void calcCost(CTNode* node, Agents& block);
  void formalizePathAgents(CTNode* node, Agents& block);
  virtual Nodes AstarSearch(Agent* a, CTNode* node);
  Constraint getConstraintsForAgent(CTNode* ctNode, Agent* a);
  Constraint getConstraints(CTNode* ctNode);
  bool checkAgents(Paths& paths, std::vector<Agents>& whole);
  virtual bool solvePart(Paths& paths, Agents& block);
  bool isDuplicatedConflict(Constraint &constraint, Conflict* c);
  bool isDuplicatedConflict(Conflict* c1, Conflict* c2);
  bool isDuplicatedCTNode(CTNode* newCT, std::vector<CTNode*> &cts);
  int countCollisions(CTNode* c, Paths& paths);
  void setCKey(Conflict* c);
  std::string getCKey(Node* s, Node* g, Constraint &constraints);
  bool validShorcut(Agent* a, AN* n, Node* g,
                    Constraint &constraints,
                    Nodes &tmpPath);

public:
  CBS(Problem* _P);
  CBS(Problem* _P, bool _ID);
  ~CBS();

  bool solve();

  virtual std::string logStr();
};
