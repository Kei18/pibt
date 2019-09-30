/*
 * ecbs.h
 *
 * Purpose: improved Enhanced Conflict-based Search
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#pragma once
#include "ecbs.h"


class iECBS : public ECBS {
protected:
  float w2;
  std::string highwayFile;
  std::unordered_map<std::string, float> highway;

  Eigen::MatrixXf wdists;

  void init();
  float highwayCost(Node* s, Node* g);
  Nodes AstarSearch(Agent* a, CTNode* node);

public:
  iECBS(Problem* _P, float _w);
  iECBS(Problem* _P, float _w, bool _ID);
  ~iECBS();

  std::string logStr();
};
