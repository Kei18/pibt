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
#include "../util/util.h"


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

bool ECBS::solvePart(Paths& paths, Agents& block) {
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
                         [](Nodes p) { return !p.empty(); });

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
    node = table[keyF];

    constraints = valid(node, block);
    if (constraints.empty()) break;

    updateMin = (key == keyF);
    auto itrP = std::find(OPEN.begin(), OPEN.end(), keyF);
    OPEN.erase(itrP);

    for (auto constraint : constraints) {
      CTNode* newNode = new CTNode { constraint, node->paths,
                                     0, node, true, {}, 0 };
      // formating
      Node* g;
      Nodes p;
      for (int i = 0; i < node->paths.size(); ++i) {
        p = newNode->paths[i];
        if (p.empty()) continue;
        g = *(p.end() - 1);
        while (*(p.end() - 1) == g) p.erase(p.end() - 1);
        p.push_back(g);
        node->paths[i] = p;
      }
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

void ECBS::invoke(CTNode* node, Agents& block) {
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
  formalizePathAgents(node, block);
}

int ECBS::h3(Agent* a, Nodes &p1, Paths &paths) {
  if (p1.empty()) return 0;

  int collision = 0;
  Nodes p2;

  for (int i = 0; i < paths.size(); ++i) {
    if (a->getId() == i) continue;
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
  Nodes p1, p2;

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
          // goal pos
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
        // intersection
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

struct Fib_FN { // Forcul Node for Fibonacci heap
  Fib_FN(AN* _node, int _h): node(_node), h(_h) {}
  AN* node;
  int h;

  bool operator<(const Fib_FN& other) const {
    if (h != other.h) {
      return h > other.h;
    } else {
      if (node->f != other.node->f) {
        return node->f > other.node->f;
      } else {
        return node->g < other.node->g;
      }
    }
  }
};

Nodes ECBS::AstarSearch(Agent* a, CTNode* node) {
  Constraint constraints = getConstraintsForAgent(node, a);

  Node* _s = a->getNode();
  Node* _g = a->getGoal();

  Nodes path, tmpPath, C;  // return

  // ==== fast implementation ====
  // constraint free
  if (constraints.empty()) {
    path = G->getPath(_s, _g);
    table_fmin.at(a->getId()) = path.size() - 1;
    return path;
  }

  // goal condition
  bool existGoalConstraint = false;
  int timeGoalConstraint = 0;
  for (auto c: constraints) {
    if (c->onNode && c->v == _g && c->t > timeGoalConstraint) {
      existGoalConstraint = true;
      timeGoalConstraint = c->t;
    }
  }
  // =============================

  auto paths = node->paths;

  int f, g;
  float ub;
  int ubori;
  std::string key, keyM, keyF;
  bool invalid = true;

  boost::heap::fibonacci_heap<Fib_AN> OPEN;
  std::unordered_map<std::string, boost::heap::fibonacci_heap<Fib_AN>::handle_type> SEARCHED;
  std::unordered_set<std::string> CLOSE;  // key
  AN* n = new AN { _s, 0, pathDist(_s, _g), nullptr };
  auto handle = OPEN.push(Fib_AN(n));
  key = getKey(n);
  SEARCHED.emplace(key, handle);

  bool updateMin = true;
  std::unordered_map<std::string, int> table_conflict;
  table_conflict.emplace(key, 0);
  // FOCUL
  boost::heap::fibonacci_heap<Fib_FN> FOCUL;
  std::unordered_map<std::string,
                     boost::heap::fibonacci_heap<Fib_FN>::handle_type> SEARCHED_F;

  while (!OPEN.empty()) {
    if (updateMin || FOCUL.empty()) {
      // argmin
      while (!OPEN.empty()
             && CLOSE.find(getKey(OPEN.top().node)) != CLOSE.end()) OPEN.pop();
      if (OPEN.empty()) break;
      n = OPEN.top().node;
      ubori = n->f;
      keyM = getKey(n);
      ub = n->f * w;
      // update focul
      FOCUL.clear();
      SEARCHED_F.clear();
      for (auto itr = OPEN.ordered_begin(); itr != OPEN.ordered_end(); ++itr) {
        AN* l = (*itr).node;
        if ((float)l->f <= ub) {
          if (CLOSE.find(getKey(l)) == CLOSE.end()) {
            key = getKey(l);
            auto handle_f = FOCUL.push(Fib_FN(l, table_conflict.at(key)));
            SEARCHED_F.emplace(key, handle_f);
          }
        } else {
          break;
        }
      }
    }

    // argmin in FOCUL
    n = FOCUL.top().node;
    key = getKey(n);
    FOCUL.pop();

    // already explored
    if (CLOSE.find(key) != CLOSE.end()) continue;

    // check goal
    if (n->v == _g) {
      if (!existGoalConstraint || timeGoalConstraint < n->g) {
        invalid = false;
        break;
      }
    }

    // update list
    updateMin = (key == keyM);
    CLOSE.emplace(key);

    // search neighbor
    C = G->neighbor(n->v);
    C.push_back(n->v);

    for (auto m : C) {
      g = n->g + 1;
      key = getKey(g, m);
      if (CLOSE.find(key) != CLOSE.end()) continue;

      // check constraints
      auto constraint = std::find_if(constraints.begin(), constraints.end(),
                                     [a, g, m, n] (Conflict* c) {
                                       if (c->a != a) return false;
                                       if (c->t != g) return false;
                                       if (c->onNode) return c->v == m;
                                       return c->v == m && c->u == n->v;
                                     });
      if (constraint != constraints.end()) continue;
      f = g + pathDist(m, _g);

      // ==== fast implementation ====
      // when field is huge, this works well
      // if (existGoalConstraint) {
      //   f = pathDist(m, _g) + timeGoalConstraint;
      // }
      // =============================

      auto itrS = SEARCHED.find(key);
      AN* l;
      bool updateH = false;
      if (itrS == SEARCHED.end()) {  // new node
        l = new AN { m, g, f, n };
        auto handle = OPEN.push(Fib_AN(l));
        SEARCHED.emplace(key, handle);
        getPartialPath(l, tmpPath);
        table_conflict.emplace(key, h3(a, tmpPath, paths));
      } else {
        auto handle = itrS->second;
        l = (*handle).node;
        if (l->f > f) {
          l->g = g;
          l->f = f;
          l->p = n;
          getPartialPath(l, tmpPath);
          table_conflict.at(key) = h3(a, tmpPath, paths);
          OPEN.increase(handle);
          updateH = true;
        }
      }

      tmpPath.clear();
      if (f <= ub) {
        auto itrSF = SEARCHED_F.find(key);
        if (itrSF == SEARCHED_F.end()) {
          auto handle_f = FOCUL.push(Fib_FN(l, table_conflict.at(key)));
          SEARCHED_F.emplace(key, handle_f);
        } else {
          if (updateH) {
            auto handle_f = itrSF->second;
            (*handle_f).h = table_conflict.at(key);
            FOCUL.increase(handle_f);
          }
        }
      }
    }
  }

  // back tracking
  int fmin = 0;
  if (!invalid) {  // check failed or not
    getPartialPath(n, path);
    fmin = ubori;
  } else {
    node->valid = false;
  }
  table_fmin.at(a->getId()) = fmin;

  return path;
}

void ECBS::getPartialPath(AN* n, Nodes &path) {
  path.clear();
  AN* m = n;
  while (m != nullptr) {
    path.push_back(m->v);
    m = m->p;
  }
  std::reverse(path.begin(), path.end());
}

std::string ECBS::logStr() {
  std::string str;
  str += "[solver] type:ECBS\n";
  str += "[solver] w:" + std::to_string(w) + "\n";
  str += "[solver] ID:" + std::to_string(ID) + "\n";
  str += Solver::logStr();
  return str;
}
