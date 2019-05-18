#pragma once

#include "solver.h"

struct CG {  // constraint of goal, for agents who have reached goals
  Agent* a;
  int t;
  Node* v;
};

class WHCA : public Solver {
private:
  bool hasWindow;  // HCA* or WHCA*
  int window;

  std::vector<CG*> CGOAL;  // goals that agents have already reached

  void init();
  Nodes getPath(Agent* a, int startTime, Paths& paths);

public:
  WHCA(Problem* _P);
  WHCA(Problem* _P, int _w);
  ~WHCA();

  bool solve();

  virtual std::string logStr();
};
