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
  Node* g;

  // initialize path
  for (auto a : A) PATHS.push_back({ a->getNode() });

  while (!P->isSolved()) {
    for (int i = 0; i < A.size(); ++i) {
      std::vector<Node*> path = getPath(A[i], t, PATHS);

      // check whether success for computing path or not
      if (path.empty()) {
        failed = true;
        break;
      }

      // reach goal
      g = A[i]->getGoal();
      if (!hasWindow && path[path.size() - 1] == A[i]->getGoal()) {
        CG* cg = new CG { A[i], (int)path.size() - 1, g };
        CGOAL.push_back(cg);
      }
      if (hasWindow && (*path.begin()) != g) {
        auto itr = std::find_if(path.begin() + 1, path.begin() + window + 1,
                                 [g](Node* v) { return v == g; });
        if (itr != path.begin() + window + 1) {  // reach goal
          int dis = std::distance(path.begin(), itr);
          CG* cg = new CG { A[i], t + dis, g };
          CGOAL.push_back(cg);
        }
      }

      PATHS[i].insert(PATHS[i].end(),
                      path.begin() + 1,
                      path.begin() + std::min(window + 1, (int)path.size()));
    }

    if (failed) break;

    formalizePath(PATHS);  // adjust path lengths
    limit = std::min(window, (int)PATHS[0].size() - t);
    for (int _t = 0; _t < limit; ++_t) {
      ++t;
      for (int i = 0; i < A.size(); ++i) A[i]->setNode(PATHS[i][t]);
      P->update();
      if (P->isSolved()) break;
    }
  }

  solveEnd();
  return !failed;
}

std::vector<Node*> WHCA::getPath(Agent* a, int startTime, Paths& paths) {
  std::vector<Node*> path;
  std::vector<Node*> C;
  Node* s = a->getNode();
  Node* g = a->getGoal();
  int f = 0;
  int t = startTime;
  std::string key;
  bool prohibited = false;
  int maxLength = getMaxLengthPaths(paths);

  // fast implementation
  std::vector<Node*> tmp = G->getPath(s, g);  // already known
  if (hasWindow) {
    while (tmp.size() <= window) tmp.push_back(g);
  } else {
    while (tmp.size() <= maxLength) tmp.push_back(g);
  }

  // check result of fast implementation
  // first, check goals
  for (auto cg : CGOAL) {
    if (cg->a == a) continue;
    for (auto v : tmp) {
      if (cg->v == v) {
        prohibited = true;
        break;
      }
    }
    if (prohibited) break;
  }
  // second, check collision and intersection
  if (!prohibited) {
    for (auto p : paths) {
      for (t = startTime + 1; t < p.size(); ++t) {
        if (t >= p.size()) break;
        if (p[t] == tmp[t-startTime]) { // collision
          prohibited = true;
          break;
        }
        if (t > startTime) {  // intersection
          if (p[t] == tmp[t-startTime-1] && p[t-1] == tmp[t-startTime]) {
            prohibited = true;
            break;
          }
        }
      }
      if (prohibited) break;
    }
  }
  // success of fast implementation
  if (!prohibited) {
    return tmp;
  }

  // normal path finding
  t = startTime;
  AN *l, *n;
  std::unordered_set<std::string> OPEN;
  std::unordered_map<std::string, AN*> table;

  l = new AN { s, true, startTime, pathDist(s, g), nullptr };
  key = getKey(t, s);
  table.emplace(key, l);
  OPEN = { key };

  bool invalid = true;
  while (!OPEN.empty()) {
    // argmin
    auto itr1 = std::min_element(OPEN.begin(), OPEN.end(),
                                 [&table](std::string a, std::string b)
                                 { auto eleA = table.at(a);
                                   auto eleB = table.at(b);
                                   if (eleA->f == eleB->f) {
                                     return eleA->t > eleB->t;
                                   }
                                   return eleA->f < eleB->f; });
    key = *itr1;
    n = table.at(key);

    // fast implementation
    if (hasWindow && t > startTime + window) {
      invalid = false;
      break;
    }

    // check goal
    if (!hasWindow) {
      if (n->v == g && n->t >= maxLength - 1) {
        invalid = false;
        break;
      }
    } else {
      if (n->v == g && n->t >= startTime + window) {
        invalid = false;
        break;
      }
    }

    // update list
    n->open = false;
    OPEN.erase(itr1);

    // search neighbor
    C = G->neighbor(n->v);
    C.push_back(n->v);

    for (auto m : C) {
      // check other paths
      t = n->t + 1;

      if (t > 0) {
        if (!(hasWindow && t > startTime + window)) {
          prohibited = false;

          for (auto p : paths) {
            if (t >= p.size()) continue;
            if (p[t] == m) { // collision
              prohibited = true;
              break;
            }
            if (p[t] == n->v && p[t-1] == m) {  // intersection
              prohibited = true;
              break;
            }
          }

          if (!prohibited) {
            for (auto cg : CGOAL) {
              if (a != cg->a && t >= cg->t && m == cg->v) {
                prohibited = true;
                break;
              }
            }
          }

          if (prohibited) continue;
        }
      }

      t = n->t + 1;
      key = getKey(t, m);
      auto itr2 = table.find(key);
      if (itr2 == table.end()) {
        table.emplace(key, new AN { m, true, t, 100000, n });
      }
      l = table.at(key);
      if (!l->open) continue;

      f = t + pathDist(m, g);
      if (l->f > f) {
        l->t = t;
        l->f = f;
        l->p = n;
      }

      OPEN.insert(key);
    }
  }

  // back tracking
  if (!invalid) {  // check failed or not
    while (n->p) {
      path.push_back(n->v);
      n = n->p;
    }
    path.push_back(s);
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
