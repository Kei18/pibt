/*
 * iecbs.cpp
 *
 * Purpose: improved Enhanced Conflict-based Search
 *
 * Cohen, L., Uras, T., Satish Kumar, T. K., Xu, H., Ayanian, N., & Koenig, S. (2016).
 * Improved solvers for bounded-suboptimal multi-agent path finding.
 *
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "iecbs.h"
#include <fstream>
#include <regex>
#include "../util/util.h"

iECBS::iECBS(Problem* _P, float _w) : ECBS(_P, _w)
{
  init();
}
iECBS::iECBS(Problem* _P, float _w, bool _ID) : ECBS(_P, _w, _ID)
{
  init();
}

iECBS::~iECBS() {}

void iECBS::init() {
  w2 = 2;
  wdists = Eigen::MatrixXf::Zero(G->getNodesNum(), G->getNodesNum());
  highwayFile = G->getMapName() + ".highway";
  std::ifstream file(highwayFile);
  if (!file) {
    std::cout << "error@iECBS::init, file "
              << highwayFile
              << " does not exist" << "\n";
    std::exit(1);
  }

  int w = G->getW();
  int x, y;
  std::string key, line;
  Node *v, *u;

  y = 0;
  while (getline(file, line)) {
    if (line.size() != w) {
      std::cout << "error@iECBS::init, width is different, "
                << "not " << line.size() << ", should be " << w << "\n";
      std::exit(1);
    }

    x = 0;
    for (auto s : line) {
      v = G->getNode(x, y);
      if (v == nullptr) {
        ++x;
        continue;
      }

      // right
      u = G->getNode(x + 1, y);
      if (u != nullptr) {
        key = std::to_string(v->getId()) + "-" + std::to_string(u->getId());
        if (s == 'r' || s == 'x' || s == 'z') {
          highway.emplace(key, 1);
        } else {
          highway.emplace(key, w2);
        }
      }

      // left
      u = G->getNode(x - 1, y);
      if (u != nullptr) {
        key = std::to_string(v->getId()) + "-" + std::to_string(u->getId());
        if (s == 'l' || s == 'y' || s == 'w') {
          highway.emplace(key, 1);
        } else {
          highway.emplace(key, w2);
        }
      }

      // up
      u = G->getNode(x, y - 1);
      if (u != nullptr) {
        key = std::to_string(v->getId()) + "-" + std::to_string(u->getId());
        if (s == 'u' || s == 'z' || s == 'w') {
          highway.emplace(key, 1);
        } else {
          highway.emplace(key, w2);
        }
      }

      // down
      u = G->getNode(x, y + 1);
      if (u != nullptr) {
        key = std::to_string(v->getId()) + "-" + std::to_string(u->getId());
        if (s == 'd' || s == 'x' || s == 'y') {
          highway.emplace(key, 1);
        } else {
          highway.emplace(key, w2);
        }
      }

      ++x;
    }
    ++y;
  }
  if (y != G->getH()) {
    std::cout << "error@iECBS::init, height is different, "
              << "not " << y + 1 << ", should be " << G->getH() << "\n";
    std::exit(1);
  }
}

struct Fib_FN_iECBS { // Forcul Node for Fibonacci heap, iECBS
  Fib_FN_iECBS(AN* _node, int _h, float _highway)
    : node(_node), h(_h), highway(_highway) {}
  AN* node;
  int h;
  float highway;

  bool operator<(const Fib_FN_iECBS & other) const {
    if (h != other.h) {
      return h > other.h;
    } else {  // h == other.h
      float cThis  = highway + (float)node->g;
      float cOther = other.highway + (float)other.node->g;
      if (cThis != cOther) {
        return cThis > cOther;
      } else {
        return node->g < other.node->g;
      }
    }
  }
};

Nodes iECBS::AstarSearch(Agent* a, CTNode* node) {
  Constraint constraints = getConstraintsForAgent(node, a);

  Node* _s = a->getNode();
  Node* _g = a->getGoal();

  Nodes path, tmpPath, C;

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
  std::string key, keyF, keyM;
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
  boost::heap::fibonacci_heap<Fib_FN_iECBS> FOCUL;
  std::unordered_map<std::string,
                     boost::heap::fibonacci_heap<Fib_FN_iECBS>::handle_type> SEARCHED_F;

  while (!OPEN.empty()) {
    // argmin
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
        if (CLOSE.find(getKey(l)) == CLOSE.end()) {
          key = getKey(l);
          auto handle_f = FOCUL.push(Fib_FN_iECBS(l, table_conflict.at(key),
                                                  highwayCost(l->v, _g)));
          SEARCHED_F.emplace(key, handle_f);
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
      if (existGoalConstraint) {
        f = pathDist(m, _g) + timeGoalConstraint;
      }
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
          auto handle_f = FOCUL.push(Fib_FN_iECBS(l, table_conflict.at(key),
                                                  highwayCost(m, _g)));
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

struct ANF {  // Astar Node for Floating representation
  Node* v;
  float g;
  float f;
  ANF* p;
};
struct Fib_ANF { // AN for Fibonacci heap
  Fib_ANF(ANF* _node): node(_node) {}
  ANF* node;

  bool operator<(const Fib_ANF& other) const {
    if (node->f != other.node->f) {
      return node->f > other.node->f;
    } else {
      return node->g < other.node->g;
    }
  }
};

float iECBS::highwayCost(Node* _s, Node* _g) {
  // ==== fast implementation ====
  if (_s == _g) return 0;

  // registered
  int dist = wdists(_s->getIndex(), _g->getIndex());
  if (dist != 0) return dist;
  // =============================

  float f, w, d;
  bool invalid = true;
  Nodes C;
  std::string keyW;

  // prepare node open hashtable
  boost::heap::fibonacci_heap<Fib_ANF> OPEN;
  std::unordered_map<int, boost::heap::fibonacci_heap<Fib_ANF>::handle_type> SEARCHED;
  std::unordered_set<int> CLOSE;
  ANF* n = new ANF { _s, 0, static_cast<float>(pathDist(_s, _g)), nullptr };
  auto handle = OPEN.push(Fib_ANF(n));
  SEARCHED.emplace(n->v->getId(), handle);

  while (!OPEN.empty()) {
    // argmin
    n = OPEN.top().node;

    // check goal condition
    if (n->v == _g) {
      invalid = false;
      break;
    }

    // ==== fast implementation ====
    dist = wdists(n->v->getIndex(), _g->getIndex());
    if (dist != 0) {
      d = dist;
      // register
      while (n->p != nullptr) {
        dist = wdists(n->v->getIndex(), _g->getIndex());
        keyW = std::to_string(n->p->v->getId()) + "-" + std::to_string(n->v->getId());
        w = highway.at(keyW);
        wdists(n->p->v->getIndex(), _g->getIndex()) = dist + w;
        n = n->p;
      }
      return d;
    }
    // =============================

    // update list
    OPEN.pop();
    CLOSE.emplace(n->v->getId());

    // search neighbor
    C = G->neighbor(n->v);

    for (auto m : C) {
      if (CLOSE.find(m->getId()) != CLOSE.end()) continue;
      keyW = std::to_string(n->v->getId()) + "-" + std::to_string(m->getId());
      w = highway.at(keyW);
      f = n->g + w + pathDist(m, _g);

      // ==== fast implementation ====
      dist = wdists(m->getIndex(), _g->getIndex());
      if (dist != 0) f = n->g + w + dist;
      // =============================

      auto itrS = SEARCHED.find(m->getId());
      if (itrS == SEARCHED.end()) {  // new node
        ANF* l = new ANF { m, n->g + w, f, n };
        auto handle = OPEN.push(Fib_ANF(l));
        SEARCHED.emplace(l->v->getId(), handle);
      } else {
        auto handle = itrS->second;
        ANF* l = (*handle).node;
        if (l->f > f) {
          l->g = n->g + w;
          l->f = f;
          l->p = n;
          OPEN.increase(handle);
        }
      }
    }
  }

  if (invalid) {
    std::cout << "error@iEPIBT::highwayCost, " << "cannot find a path : "
              << _s->getId() << " -> " << _g->getId() << "\n";
    std::exit(1);
  }

  // ==== fast implementation ====
  d = n->g;
  while (n != nullptr) {
    wdists(n->v->getIndex(), _g->getIndex()) = n->g;
    n = n->p;
  }
  // =============================
  return d;
}

std::string iECBS::logStr() {
  std::string str;
  str += "[solver] type:iECBS\n";
  str += "[solver] w:" + std::to_string(w) + "\n";
  str += "[solver] highway:" + highwayFile + "\n";
  str += "[solver] ID:" + std::to_string(ID) + "\n";
  str += Solver::logStr();
  return str;
}
