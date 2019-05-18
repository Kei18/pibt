/*
 * ecbs.cpp
 *
 * Purpose: Enhanced Conflict-based Search
 *
 * Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015).
 * Conflict-based search for optimal multi-agent pathfinding.
 *
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "ecbs.h"


ECBS::ECBS(Problem* _P, float _w) : CBS(_P, false), w(_w) {
  init();
}
ECBS::ECBS(Problem* _P, float _w, bool _ID) : CBS(_P, _ID), w(_w) {
  init();
}

ECBS::~ECBS() {}

void ECBS::init() {
  for (auto a : A) table_fmin.emplace(a->getId(), 0);
}

bool ECBS::solvePart(Paths& paths, Block& block) {
  CTNode* node;
  Constraints constraints;

  int uuid = 0;
  int key, keyF;
  std::unordered_set<int> OPEN;
  std::vector<int> FOCUL;  // for FOCUL search
  float ub;

  bool status = true;
  bool updateMin = true;

  std::vector<CTNode*> table;
  std::vector<int> table_conflict;

  CTNode* root = new CTNode { {}, {}, 0, nullptr, true, {}, 0 };
  invoke(root, block);
  OPEN.insert(uuid);
  table.push_back(root);
  table_conflict.push_back(h3(root->paths));
  ++uuid;

  bool CAT = std::any_of(paths.begin(), paths.end(),
                         [](std::vector<Node*> p) { return !p.empty(); });

  auto itrO = OPEN.begin();

  while (!OPEN.empty()) {

    if (updateMin) {
      itrO = std::min_element(OPEN.begin(), OPEN.end(),
                              [CAT, this, &paths, &table]
                              (int a, int b) {
                                CTNode* nA = table[a];
                                CTNode* nB = table[b];
                                if (CAT && nA->cost == nB->cost) {
                                  return this->countCollisions(nA, paths)
                                    < this->countCollisions(nB, paths);
                                }
                                return nA->cost < nB->cost;
                              });
    }

    key = *itrO;
    node = table[key];

    ub = node->LB * w;
    // update focul
    FOCUL.clear();
    for (auto keyO : OPEN) {
      if ((float)table[keyO]->cost <= ub) FOCUL.push_back(keyO);
    }
    auto itrF = std::min_element(FOCUL.begin(), FOCUL.end(),
                                 [&table, &table_conflict]
                                 (int a, int b) {
                                   int sA = table_conflict[a];
                                   int sB = table_conflict[b];
                                   if (sA == sB) {
                                     return table[a]->cost < table[b]->cost;
                                   }
                                   return sA < sB; });
    keyF = *itrF;
    node = table.at(keyF);

    constraints = valid(node, block);
    if (constraints.empty()) break;

    updateMin = (key == keyF);
    auto itrP = std::find(OPEN.begin(), OPEN.end(), keyF);
    OPEN.erase(itrP);

    for (auto constraint : constraints) {
      CTNode* newNode = new CTNode { constraint,
                                     node->paths, 0,
                                     node, true,
                                     {}, 0 };
      newNode->fmins = node->fmins;
      invoke(newNode, block);
      if (newNode->valid) {
        OPEN.insert(uuid);
        table.push_back(newNode);
        table_conflict.push_back(h3(newNode->paths));
        ++uuid;
      }
    }
    constraints.clear();
  }

  if (!OPEN.empty()) {  // sucssess
    for (int i = 0; i < paths.size(); ++i) {
      if (!node->paths[i].empty()) {
        paths[i] = node->paths[i];
      }
    }
    status = status && true;
  } else {
    status = false;
  }

  return status;

}

void ECBS::invoke(CTNode* node, Block& block) {
  int d;
  // calc path
  if (node->c.empty()) {  // initail
    Paths paths;
    for (int i = 0; i < A.size(); ++i) {
      paths.push_back({});
      node->fmins.push_back(0);
    }
    for (auto a : block) {
      auto itr = std::find_if(A.begin(), A.end(),
                              [a](Agent* b) { return a == b; });
      int d = std::distance(A.begin(), itr);
      paths[d] = AstarSearch(a, node);
      node->fmins[d] = table_fmin.at(a->getId());
    }
    node->paths = paths;
    node->LB = 0;
  } else {
    Agent* a;
    // error check
    if (node->paths.size() != A.size()) {
      std::cout << "error@ECBS@invoke, "
                << "path size is not equal to the size of agents" << "\n";
      std::exit(1);
    }

    for (auto c : node->c) {
      a = c->a;
      auto itr = std::find_if(A.begin(), A.end(),
                              [a](Agent* b) { return a == b; });
      if (itr == A.end()) {
        std::cout << "error@ECBS@invoke, cannot find1 agent" << "\n";
        std::exit(1);
      }
      d = std::distance(A.begin(), itr);
      node->paths[d].clear();
    }

    for (auto c : node->c) {
      a = c->a;
      auto itr = std::find_if(A.begin(), A.end(),
                              [a](Agent* b) { return a == b; });
      if (itr == A.end()) {
        std::cout << "error@ECBS@invoke, cannot find2 agent" << "\n";
        std::exit(1);
      }
      d = std::distance(A.begin(), itr);
      node->paths[d] = AstarSearch(a, node);
      node->fmins[d] = table_fmin.at(a->getId());
    }
  }
  for (auto i : node->fmins) node->LB += i;

  if (!node->valid) return;
  calcCost(node, block);
  formalizePathBlock(node, block);
}

int ECBS::h3(std::vector<Node*> &p1, Paths &paths) {
  if (p1.empty()) return 0;

  int collision = 0;
  std::vector<Node*> p2;

  for (int i = 0; i < paths.size(); ++i) {
    p2 = paths[i];
    if (p2.empty()) continue;
    for (int t = 0; t < p1.size(); ++t) {
      if (t >= p2.size()) {
        if (p1[t] == p2[p2.size() - 1]) {
          ++collision;
          break;
        }
        continue;
      }
      if (p1[t] == p2[t]) {  // collision
        ++collision;
        break;
      }
      if (t > 0 && p1[t-1] == p2[t] && p1[t] == p2[t-1]) {
        ++collision;
        break;
      }
    }
  }

  // error check
  if (collision > A.size() * (A.size() - 1) / 2) {
    std::cout << "error@ECBS::h3, invalid value, " << collision << "\n";
    std::exit(1);
  }

  return collision;
}

int ECBS::h3(Paths &paths) {

  int collision = 0;
  std::vector<Node*> p1, p2;

  for (int i = 0; i < paths.size(); ++i) {
    for (int j = i + 1; j < paths.size(); ++j) {
      if (paths[i].size() >= paths[j].size()) {
        p1 = paths[i];
        p2 = paths[j];
      } else {
        p1 = paths[j];
        p2 = paths[i];
      }

      if (p2.empty()) continue;

      for (int t = 0; t < p1.size(); ++t) {
        if (t >= p2.size()) {
          if (p1[t] == p2[p2.size() - 1]) {
            ++collision;
            break;
          }
          continue;
        }
        if (p1[t] == p2[t]) {  // collision
          ++collision;
          break;
        }
        if (t > 0 && p1[t-1] == p2[t] && p1[t] == p2[t-1]) {
          ++collision;
          break;
        }
      }
    }
  }

  // error check
  if (collision > A.size() * (A.size() - 1) / 2) {
    std::cout << "error@ECBS::h3, invalid value, " << collision << "\n";
    std::exit(1);
  }

  return collision;
}

std::vector<Node*> ECBS::AstarSearch(Agent* a, CTNode* node) {

  Constraint constraints = getConstraitnsForAgent(node, a);

  std::vector<Node*> path;  // return
  auto paths = node->paths;
  Node* s = a->getNode();
  Node* g = a->getGoal();

  if (constraints.empty()) {
    path = G->getPath(s, g);
    table_fmin.at(a->getId()) = path.size() - 1;
    return path;
  }

  int t = 0;
  std::vector<Node*> C;  // candidates
  std::vector<Node*> tmpPath;  // for count conflict
  int f = 100000;
  float ub;
  bool updateMin = true;
  std::string key, keyF;
  AN *l, *n;
  std::unordered_set<std::string> OPEN;
  std::vector<std::string> FOCUL;

  std::unordered_map<std::string, AN*> table;
  std::unordered_map<std::string, int> table_conflict;

  l = new AN { s, true, 0, 0, nullptr };
  key = getKey(t, s);
  table.emplace(key, l);
  table_conflict.emplace(key, 0);
  OPEN.insert(key);
  auto itrO = OPEN.begin();

  bool invalid = true;
  while (!OPEN.empty()) {
    // argmin
    if (updateMin) {
      itrO = std::min_element(OPEN.begin(), OPEN.end(),
                              [&table](std::string a, std::string b)
                              { return table.at(a)->f < table.at(b)->f; });
    }
    key = *itrO;
    n = table.at(key);

    ub = n->f * w;
    // update focul
    FOCUL.clear();
    for (auto keyO : OPEN) {
      if ((float)table.at(keyO)->f <= ub) FOCUL.push_back(keyO);
    }
    auto itrF = std::min_element(FOCUL.begin(), FOCUL.end(),
                                 [&table, &table_conflict]
                                 (std::string a, std::string b) {
                                   int sA = table_conflict.at(a);
                                   int sB = table_conflict.at(b);
                                   if (sA == sB) {
                                     auto eleA = table.at(a);
                                     auto eleB = table.at(b);
                                     if (eleA->f == eleB->f) {
                                       return eleA->t > eleB->t;
                                     }
                                     return eleA->f < eleB->f;
                                   }
                                   return sA < sB; });
    keyF = *itrF;
    n = table.at(keyF);

    // check goal
    if (n->v == g) {
      auto check = std::find_if(constraints.begin(), constraints.end(),
                                [n] (Conflict* c)
                                { return (c->v == n->v) && (c->t >= n->t); });
      if (check == constraints.end()) {
        invalid = false;
        break;
      }
    }

    // update list
    n->open = false;
    updateMin = (key == keyF);
    auto itrP = std::find(OPEN.begin(), OPEN.end(), keyF);
    if (itrP == OPEN.end()) {
      std::cout << "error@ECBS::AstarSearch, cannot find itrP" << "\n";
      std::exit(1);
    }
    OPEN.erase(itrP);

    // search neighbor
    C = G->neighbor(n->v);
    C.push_back(n->v);

    for (auto m : C) {
      // check constraints
      t = n->t + 1;
      auto constraint = std::find_if(constraints.begin(), constraints.end(),
                                     [a, t, m](Conflict* c) {
                                       return (c->a == a)
                                         && (c->t == t) && (c->v == m); });
      if (constraint != constraints.end()) continue;

      key = getKey(t, m);
      f = t + pathDist(m, g);
      auto itr2 = table.find(key);
      if (itr2 == table.end()) {
        l = new AN { m, true, t, f, n };
        table.emplace(key, l);
        tmpPath = getPartialPath(l);
        table_conflict.emplace(key, h3(tmpPath, paths));
      } else {
        l = table.at(key);
        if (!l->open) continue;
      }

      if (l->f > f) {
        l->t = t;
        l->f = f;
        l->p = n;
        tmpPath = getPartialPath(l);
        table_conflict.emplace(key, h3(tmpPath, paths));
      }

      tmpPath.clear();
      OPEN.insert(key);
    }
  }

  // back tracking
  if (!invalid) {  // check failed or not
    path = getPartialPath(n);
  } else {
    node->valid = false;
  }

  int fmin;
  if (path.empty()) {
    fmin = 0;
  } else {
    fmin = table.at(*itrO)->f;
  }
  table_fmin.at(a->getId()) = fmin;

  return path;
}

std::vector<Node*> ECBS::getPartialPath(AN* n) {
  std::vector<Node*> path;
  AN* m = n;
  while (m->p) {
    path.push_back(m->v);
    m = m->p;
  }
  path.push_back(m->v);
  std::reverse(path.begin(), path.end());
  return path;
}

std::string ECBS::logStr() {
  std::string str;
  str += "[solver] type:ECBS\n";
  str += "[solver] w:" + std::to_string(w) + "\n";
  str += Solver::logStr();
  return str;
}
