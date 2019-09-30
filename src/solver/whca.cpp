/*
 * whca.cpp
 *
 * Purpose: WHCA*
 *
 * Silver, D. (2005).
 * Cooperative pathfinding.
 *
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "whca.h"
#include <algorithm>


// HCA*
WHCA::WHCA(Problem* _P) : Solver(_P) {
  hasWindow = false;
  window = 10000;
  init();
}

WHCA::WHCA(Problem* _P, int _w) : Solver(_P), window(_w) {
  hasWindow = true;
  init();
}

WHCA::~WHCA() {
  for (auto cg : CGOAL) delete cg;
  CGOAL.clear();
}

void WHCA::init() {
  G->setRegFlg(true);
}

bool WHCA::solve() {
  solveStart();

  Paths PATHS;  // paths for all agents
  int t = 0;
  int limit;
  bool failed = false;

  // initialize path
  for (auto a : A) PATHS.push_back({ a->getNode() });

  while (!P->isSolved()) {
    for (int i = 0; i < A.size(); ++i) {
      Nodes path = getPath(A[i], t, PATHS);

      // check whether success for computing path or not
      if (path.empty()) {
        failed = true;
        break;
      }

      // reach goal
      if (!hasWindow) {
        CG* cg = new CG { A[i], (int)path.size() - 1, A[i]->getGoal() };
        CGOAL.push_back(cg);
      }

      PATHS[i].insert(PATHS[i].end(), path.begin() + 1, path.end());
    }

    if (failed) break;

    formalizePath(PATHS);
    limit = std::min(window, (int)PATHS[0].size() - t);
    for (int _t = 0; _t < limit; ++_t) {
      ++t;
      for (int i = 0; i < A.size(); ++i) A[i]->setNode(PATHS[i][t]);
      P->update();
      if (P->isSolved()) break;
      if (P->getTimestep() >= P->getTimestepLimit()) break;
    }

    if (P->getTimestep() >= P->getTimestepLimit()) break;
  }

  solveEnd();
  return !failed;
}

Nodes WHCA::getPath(Agent* a, int startTime, Paths& paths) {

  Node* _s = a->getNode();
  Node* _g = a->getGoal();

  Nodes path, tmpPath, C;

  int f, g;
  std::string key;
  bool prohibited = false;
  int maxLength = getMaxLengthPaths(paths);

  // ==== fast implementation ====
  tmpPath = G->getPath(_s, _g);
  if (hasWindow) {
    while (tmpPath.size() <= window + 1) tmpPath.push_back(_g);
    while (tmpPath.size() != window + 1) tmpPath.pop_back();
  } else {
    while (tmpPath.size() <= maxLength) tmpPath.push_back(_g);
  }
  // check result of fast implementation
  // first, check goals
  if (!hasWindow) {
    for (auto cg : CGOAL) {
      if (cg->a == a) continue;
      for (auto v : tmpPath) {
        if (cg->v == v) {
          prohibited = true;
          break;
        }
      }
      if (prohibited) break;
    }
  }
  // second, check collision and intersection
  if (!prohibited) {
    int limit;
    for (auto p : paths) {
      limit = p.size();
      if (hasWindow) limit = startTime + window + 1;
      for (int t = startTime + 1; t < limit; ++t) {
        if (t >= p.size()) break;
        if (p[t] == tmpPath[t-startTime]) { // collision
          prohibited = true;
          break;
        }
        if (t > startTime) {  // intersection
          if (p[t] == tmpPath[t-startTime-1] && p[t-1] == tmpPath[t-startTime]) {
            prohibited = true;
            break;
          }
        }
      }
      if (prohibited) break;
    }
  }
  // success of fast implementation
  if (!prohibited) return tmpPath;
  tmpPath.clear();
  // =============================

  // normal path finding
  bool invalid = true;

  boost::heap::fibonacci_heap<Fib_AN> OPEN;
  std::unordered_map<std::string, boost::heap::fibonacci_heap<Fib_AN>::handle_type> SEARCHED;
  std::unordered_set<std::string> CLOSE;  // key
  AN* n = new AN { _s, startTime, pathDist(_s, _g), nullptr };
  auto handle = OPEN.push(Fib_AN(n));
  key = getKey(n);
  SEARCHED.emplace(key, handle);

  while (!OPEN.empty()) {
    // argmin
    n = OPEN.top().node;

    // check termination condition
    if (!hasWindow) {  // non-window
      if (n->v == _g && n->g >= maxLength - 1) {
        invalid = false;
        break;
      }
    } else {  // windowed
      if (n->g >= startTime + window) {
        invalid = false;
        break;
      }
    }

    // update list
    OPEN.pop();
    CLOSE.emplace(getKey(n));

    // search neighbor
    C = G->neighbor(n->v);
    C.push_back(n->v);

    for (auto m : C) {
      // check other paths
      g = n->g + 1;
      key = getKey(g, m);
      if (CLOSE.find(key) != CLOSE.end()) continue;

      // check collision
      prohibited = false;
      for (auto p : paths) {
        if (g >= p.size()) continue;
        if (p[g] == m) {  // vertex collision
          prohibited = true;
          break;
        }
        if (p[g] == n->v && p[g-1] == m) {  // swap collision
          prohibited = true;
          break;
        }
      }
      if (prohibited) continue;

      // collsiion at goal
      if (!hasWindow) {
        for (auto cg : CGOAL) {  // collection of goals
          if (a != cg->a && g >= cg->t && m == cg->v) {
            prohibited = true;
            break;
          }
        }
        if (prohibited) continue;
      }

      f = g + pathDist(m, _g);

      auto itrS = SEARCHED.find(key);
      if (itrS == SEARCHED.end()) {  // new node
        AN* l = new AN { m, g, f, n };
        auto handle = OPEN.push(Fib_AN(l));
        SEARCHED.emplace(key, handle);
      } else {
        auto handle = itrS->second;
        AN* l = (*handle).node;
        if (l->f > f) {
          l->g = g;
          l->f = f;
          l->p = n;
        }
        OPEN.increase(handle);
      }
    }
  }

  // back tracking
  if (!invalid) {  // check failed or not
    while (n != nullptr) {
      path.push_back(n->v);
      n = n->p;
    }
    std::reverse(path.begin(), path.end());
  }

  return path;
}

std::string WHCA::logStr() {
  std::string str;
  str += "[solver] type:WHCA-" + std::to_string(window) + "\n";
  str += Solver::logStr();
  return str;
}
