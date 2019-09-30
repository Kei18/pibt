#pragma once

#include "../problem/problem.h"
#include <vector>
#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <boost/heap/fibonacci_heap.hpp>


class Solver {
private:
  double elapsedTime;

protected:
  Problem* P;
  std::mt19937* MT;

  Agents A;
  Graph* G;

  Eigen::MatrixXi dists;

  void init();
  int getMaxLengthPaths(Paths& paths);
  void formalizePath(Paths& paths);
  int pathDist(Node* v, Node* u);
  int pathDist(Node* s, Node* g, Nodes &prohibited);
  std::vector<Agents> findAgentBlock();
  static std::string getKey(int t, Node* v);
  static std::string getKey(AN* n);

  virtual void solveStart();
  virtual void solveEnd();
  std::chrono::system_clock::time_point startT;
  std::chrono::system_clock::time_point endT;

public:
  Solver(Problem* _P);
  Solver(Problem* _P, std::mt19937* _MT);
  ~Solver();

  void WarshallFloyd();

  virtual bool solve() { return false; };
  double getElapsed() { return elapsedTime; };

  virtual std::string logStr();
};
