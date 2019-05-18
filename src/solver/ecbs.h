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

  virtual void init();
  int h3(Paths &paths);
  int h3(Agent* a, Nodes& p1, Paths &paths);
  bool solvePart(Paths& paths, Agents& block);
  void invoke(CTNode* node, Agents& block);
  virtual Nodes AstarSearch(Agent* a, CTNode* node);
  Nodes getPartialPath(AN* n);

public:
  ECBS(Problem* _P, float _w);
  ECBS(Problem* _P, float _w, bool _ID);
  ~ECBS();

  virtual std::string logStr();
};
