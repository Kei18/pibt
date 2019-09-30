/*
 * pt.cpp
 *
 * Purpose: Token Passing
 *
 * Ma, H., Li, J., Kumar, T. K. S., & Koenig, S. (2017).
 * Lifelong Multi-Agent Path Finding for Online Pickup and Delivery Tasks.
 *
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "tp.h"
#include <iomanip>
#include "../util/util.h"


TP::TP(Problem* _P, Nodes _ep) : Solver(_P) {
  endpoints = _ep;
  init();
}

TP::TP(Problem* _P, std::vector<int> _ep) : Solver(_P) {
  for (auto i : _ep) endpoints.push_back(G->getNode(i));
  init();
}

void TP::init() {
  timestep = 0;
  status = true;
  P->setAutoAssignement(false);

  G->setRegFlg(true);

  // initialize
  for (auto a : A) paths.push_back({ a->getNode() });
}

TP::~TP() {
  endpoints.clear();
}

bool TP::solve() {
  solveStart();

  while (!P->isSolved()) {
    ++timestep;
    update();

    if (!status) {
      solveEnd();
      return false;  // failed
    }

    P->update();
    if (P->getTimestep() >= P->getTimestepLimit()) break;
  }

  solveEnd();
  return true;
}

void TP::update() {
  Agent* a;
  Node* v;
  Task* tau;
  std::vector<Task*> tasks;

  for (int i = 0; i < A.size(); ++i) {
    a = A[i];

    if (paths[i].size() <= timestep) {
      v = a->getNode();
      tasks = getExecutableTask(a, timestep);  // l.7

      if (!tasks.empty()) {  // l.8
        tau = getNearestTask(a, tasks);  // l.9

        a->setTask(tau);  // l.10
        P->assign(tau);   // l.11
        if (tau->getG()[0] == a->getNode()) {  // when current place is pickup node
          tau->update(a->getNode());
        }

        updatePath1(a, timestep);  // l.12
      } else if (!shouldAvoid(a, timestep)) {  // l.13
        paths[i].push_back(v);  // l.14
      } else {
        updatePath2(a, timestep);  // l.16
      }
    }

    a->setNode(paths[i][timestep]);
  }
}

bool TP::shouldAvoid(Agent* a, int timestep) {
  Node* v = a->getNode();
  Node* g;

  for (auto tau : P->getT()) {
    g = tau->getG()[tau->getG().size() - 1];  // delivery points
    if (g == v) return true;
  }
  return false;
}

void TP::updatePath1(Agent* a, int startTime) {
  auto itr = std::find(A.begin(), A.end(), a);
  int i = std::distance(A.begin(), itr);

  Nodes gs = a->getTask()->getG();
  a->setGoal(gs[gs.size() - 1]);  // delivery point

  Nodes p;
  Nodes path = { a->getNode() };

  if (gs.size() == 2) {
    path = getPickDelivPath(a, a->getNode(), gs[0], gs[1], startTime, true);
  } else if (gs.size() == 1) {
    path = getPath(a, a->getNode(), gs[0], startTime, gs[0]);
  }

  paths[i].insert(paths[i].end(), path.begin() + 1, path.end());
}

void TP::updatePath2(Agent* a, int startTime) {
  Nodes candidates;
  std::vector<Task*> T = P->getT();
  bool contained;

  // create candidates
  for (auto v : endpoints) {
    contained = std::any_of(T.begin(), T.end(),
                             [v](Task* tau) {
                               return tau->getG()[tau->getG().size() - 1] == v; });
    if (contained) continue;
    contained = std::any_of(paths.begin(), paths.end(),
                            [v](Nodes path) {
                              return path[path.size() - 1] == v; });
    if (contained) continue;
    candidates.push_back(v);
  }

  if (candidates.empty()) {  // failed
    std::cout << "updatePath2, candidates are empty" << a->getId() << "\n";
    status = false;
    return;
  }

  // choose candidates
  Node* u = a->getNode();
  Graph* _G = G;
  auto itr1 = std::min_element(candidates.begin(), candidates.end(),
                               [u, _G](Node* v1, Node* v2) {
                                 return _G->dist(u, v1) < _G->dist(u, v2); });
  a->setGoal(*itr1);
  Nodes path = getPath(a, startTime);

  if (path.empty()) {
    std::cout << "updatePath2, path is empty" << a->getId() << "\n";
    status = false;
    return;
  }

  auto itr2 = std::find(A.begin(), A.end(), a);
  int i = std::distance(A.begin(), itr2);
  paths[i].insert(paths[i].end(), path.begin() + 1, path.end());
}

std::vector<Task*> TP::getExecutableTask(Agent* a, int timestep) {
  std::vector<Task*> tasks;
  Nodes gs;
  bool notContained;
  Node* v = a->getNode();
  Node* u;

  for (auto tau : P->getT()) {
    notContained = true;
    for (auto path : paths) {
      u = path[path.size() - 1];  // end of the path
      if (u == v) continue;
      gs = tau->getG();
      notContained &= std::none_of(gs.begin(), gs.end(),
                                   [u] (Node* k) { return u == k; });
      if (!notContained) break;
    }
    if (notContained) tasks.push_back(tau);
  }

  return tasks;
}


Task* TP::getNearestTask(Agent* a, std::vector<Task*>& tasks) {
  if (tasks.empty()) {
    std::cout << "Error@TP::getNearestTask, cannot get task" << "\n";
    exit(1);
  }
  Node* v = a->getNode();
  Graph* _G = G;
  auto itr = std::min_element(tasks.begin(), tasks.end(),
                              [v, _G](Task* tau1, Task* tau2) {
                                return _G->dist(v, tau1->getG()[0])
                                  < _G->dist(v, tau2->getG()[0]); });
  return *itr;
}

Nodes TP::getPath(Agent* a, int startTime) {
  return getPath(a, a->getNode(), a->getGoal(), startTime, true);
}

struct AN_OLD {  // does not use fibnacci
  Node* v;
  bool open;
  int t;
  int f;
  AN_OLD* p;
};

Nodes TP::getPath(Agent* a,
                  Node* s,
                  Node* g,
                  int startTime,
                  bool futureCollision)
{
  Nodes path;
  Nodes C;
  AN_OLD *l, *n;
  int t, f, cost;
  std::string key;
  bool prohibited, goalCheck;

  Nodes pathends;
  Node* v;
  for (auto p : paths) {
    v = p[p.size() - 1];
    if (v != s && v != a->getNode()) pathends.push_back(v);
  }

  std::unordered_set<std::string> OPEN;
  std::unordered_map<std::string, AN_OLD*> table;

  t = startTime - 1;
  l = new AN_OLD { s, true, startTime - 1, pathDist(s, g), nullptr };
  key = getKey(t, s);
  table.emplace(key, l);
  OPEN = { key };

  bool invalid = true;
  while (!OPEN.empty()) {
    // argmin
    auto itr1 = std::min_element(OPEN.begin(), OPEN.end(),
                                 [&table] (std::string a, std::string b)
                                 { auto eleA = table.at(a);
                                   auto eleB = table.at(b);
                                   if (eleA->f == eleB->f) {
                                     return eleA->t > eleB->t;
                                   }
                                   return eleA->f < eleB->f;
                                 });
    key = *itr1;
    n = table.at(key);
    t = n->t;

    // check goal
    if (n->v == g) {
      goalCheck = true;

      if (futureCollision) {
        for (auto p : paths) {
          if (t + 1 >= p.size()) continue;

          for (auto itr = p.begin() + t + 1; itr < p.end(); ++itr) {
            if ((*itr) == n->v) {  // collision in future
              goalCheck = false;
              break;
            }
          }
          if (!goalCheck) break;
        }
      }

      if (goalCheck) {
        invalid = false;
        break;
      }
    }

    // update list
    n->open = false;
    OPEN.erase(itr1);

    // search neighbor
    C = { n->v };
    for (auto u : G->neighbor(n->v)) {
      if (!inArray(u, pathends)) C.push_back(u);
    }

    for (auto m : C) {
      t = n->t + 1;

      if (t > 0) {
        prohibited = false;

        for (auto p : paths) {
          if (t + 1 > p.size()) continue;
          if (p[t] == m) { // collision
            prohibited = true;
            break;
          }
          if (p[t] == n->v && p[t-1] == m) {  // intersection
            prohibited = true;
            break;
          }
        }

        if (prohibited) continue;
      }

      key = getKey(t, m);
      auto itr2 = table.find(key);
      if (itr2 == table.end()) {
        table.emplace(key, new AN_OLD { m, true, t, 100000, n });
      }
      l = table.at(key);
      if (!l->open) continue;

      cost = pathDist(m, g);
      f = t + cost;

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

struct AN2 {
  Node* v;
  Node* g;
  bool open;
  int t;
  int f;
  AN2* p;
};

Nodes TP::getPickDelivPath(Agent* a,
                           Node* s,
                           Node* g1,
                           Node* g2,
                           int startTime,
                           bool futureCollision)
{
  Nodes path;
  Nodes C;
  AN2 *l, *n;
  int t, f, cost;
  std::string key;
  bool prohibited, goalCheck;

  Nodes pathends;
  Node* v;
  for (auto p : paths) {
    v = p[p.size() - 1];
    if (v != s && v != a->getNode()) pathends.push_back(v);
  }
  int cost_g1tog2 = pathDist(g1, g2, pathends);

  std::unordered_set<std::string> OPEN;
  std::unordered_map<std::string, AN2*> table;

  t = startTime - 1;
  l = new AN2 { s, g1, true, startTime - 1,
                pathDist(s, g1) + cost_g1tog2,
                nullptr };
  key = getKey(t, s);
  table.emplace(key, l);
  OPEN = { key };

  while (!OPEN.empty()) {
    // argmin
    auto itr1 = std::min_element(OPEN.begin(), OPEN.end(),
                                 [&table] (std::string a, std::string b)
                                 { auto eleA = table.at(a);
                                   auto eleB = table.at(b);
                                   if (eleA->f == eleB->f) {
                                     return eleA->t > eleB->t;
                                   }
                                   return eleA->f < eleB->f;
                                 });
    key = *itr1;
    n = table.at(key);
    t = n->t;

    // check goal
    if (n->v == n->g && n->g == g2) {
      goalCheck = true;

      if (futureCollision) {
        for (auto p : paths) {
          if (t + 1 >= p.size()) continue;

          for (auto itr = p.begin() + t + 1; itr < p.end(); ++itr) {
            if ((*itr) == n->v) {  // collision in future
              goalCheck = false;
              break;
            }
          }
          if (!goalCheck) break;
        }
      }

      if (goalCheck) break;

    } else if (n->v == n->g && n->g == g1) {
      n->g = g2;  // next target
    }

    // update list
    n->open = false;
    OPEN.erase(itr1);

    // search neighbor
    C = { n->v };
    for (auto u : G->neighbor(n->v)) {
      if (!inArray(u, pathends)) C.push_back(u);
    }

    for (auto m : C) {
      t = n->t + 1;

      if (t > 0) {
        prohibited = false;

        for (auto p : paths) {
          if (t + 1 > p.size()) continue;
          if (p[t] == m) { // collision
            prohibited = true;
            break;
          }
          if (p[t] == n->v && p[t-1] == m) {  // intersection
            prohibited = true;
            break;
          }
        }

        if (prohibited) continue;
      }

      key = getKey(t, m);
      auto itr2 = table.find(key);
      if (itr2 == table.end()) {
        table.emplace(key, new AN2 { m, n->g, true, t, 100000, n });
      }
      l = table.at(key);
      if (!l->open) continue;

      // cost = pathDist(m, n->g, pathends);
      cost = pathDist(m, n->g);
      f = t + cost;
      if (n->g == g1) f += cost_g1tog2;

      if (l->f > f) {
        l->t = t;
        l->f = f;
        l->p = n;
      }

      OPEN.insert(key);
    }
  }

  // back tracking
  if (n->v == g2) {  // check failed or not
    while (n->p) {
      path.push_back(n->v);
      n = n->p;
    }
    path.push_back(s);
    std::reverse(path.begin(), path.end());
  }

  return path;
}

std::string TP::logStr() {
  std::string str;
  str += "[solver] type:TP\n";
  str += Solver::logStr();
  return str;
}
