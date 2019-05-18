/*
 * ecbs.h
 *
 * Purpose: Enhanced Conflict-based Search
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#pragma once
#include "cbs.h"


class ECBS : public CBS {
protected:
  float w;  // sub-optimal factor
  std::unordered_map<int, int> table_fmin;  // record min of f-value

  void init();
  int h3(Paths &paths);
  int h3(std::vector<Node*>& p1, Paths &paths);
  bool solvePart(Paths& paths, Block& block);
  void invoke(CTNode* node, Block& block);
  std::vector<Node*> AstarSearch(Agent* a, CTNode* node);
  std::vector<Node*> getPartialPath(AN* n);

public:
  ECBS(Problem* _P, float _w);
  ECBS(Problem* _P, float _w, bool _ID);
  ~ECBS();

  std::string logStr();
};
