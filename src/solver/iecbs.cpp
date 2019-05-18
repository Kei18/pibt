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
  wdists = Eigen::MatrixXi::Zero(G->getNodesNum(), G->getNodesNum());
  highwayFile = G->getMapName() + ".highway";
  std::ifstream file(highwayFile);
  if (!file) {
    std::cout << "error@iECBS::init, file "
              << highwayFile + ".highway"
              << " does not exist" << "\n";
    std::exit(1);
  }

  int w = G->getW();
  int h = 0;
  int id = 0;
  std::string key, line;

  while (getline(file, line)) {
    ++h;
    if (line.size() != w) {
      std::cout << "error@iECBS::init, width is different, "
                << "not " << line.size() << ", should be " << w << "\n";
      std::exit(1);
    }

    for (auto s : line) {
      if (!G->existNode(id)) {
        ++id;
        continue;
      }

      // right
      if (G->existNode(id+1)) {
        key = std::to_string(id) + "-" + std::to_string(id+1);
        if (s == 'r' || s == 'x' || s == 'z') {
          highway.emplace(key, 1);
        } else {
          highway.emplace(key, w2);
        }
      }

      // left
      if (G->existNode(id-1)) {
        key = std::to_string(id) + "-" + std::to_string(id-1);
        if (s == 'l' || s == 'y' || s == 'w') {
          highway.emplace(key, 1);
        } else {
          highway.emplace(key, w2);
        }
      }

      // up
      if (G->existNode(id-w)) {
        key = std::to_string(id) + "-" + std::to_string(id-w);
        if (s == 'u' || s == 'z' || s == 'w') {
          highway.emplace(key, 1);
        } else {
          highway.emplace(key, w2);
        }
      }

      // down
      if (G->existNode(id+w)) {
        key = std::to_string(id) + "-" + std::to_string(id+w);
        if (s == 'd' || s == 'x' || s == 'y') {
          highway.emplace(key, 1);
        } else {
          highway.emplace(key, w2);
        }
      }

      ++id;
    }
  }
  if (h != G->getH()) {
    std::cout << "error@iECBS::init, height is different, "
              << "not " << h << ", should be " << G->getH() << "\n";
    std::exit(1);
  }
}

Nodes iECBS::AstarSearch(Agent* a, CTNode* node) {

  Constraint constraints = getConstraintsForAgent(node, a);

  Nodes path;  // return
  auto paths = node->paths;
  Node* s = a->getNode();
  Node* g = a->getGoal();

  if (constraints.empty()) {
    path = G->getPath(s, g);
    table_fmin.at(a->getId()) = path.size() - 1;
    return path;
  }

  int t = 0;
  Nodes C;  // candidates
  Nodes tmpPath;  // for count conflict
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
                                 [&table, &table_conflict, this, g]
                                 (std::string a, std::string b) {
                                   int sA = table_conflict.at(a);
                                   int sB = table_conflict.at(b);
                                   if (sA == sB) {
                                     auto eleA = table.at(a);
                                     auto eleB = table.at(b);
                                     int cA = this->highwayCost(eleA->v, g) + eleA->t;
                                     int cB = this->highwayCost(eleB->v, g) + eleB->t;
                                     if (cA == cB) {
                                       return eleA->t > eleB->t;
                                     }
                                     return cA < cB;
                                   }
                                   return sA < sB; });
    keyF = *itrF;
    n = table.at(keyF);

    // check goal
    if (n->v == g) {
      auto check = std::find_if(constraints.begin(), constraints.end(),
                                [n] (Conflict* c) {
                                  return c->onNode
                                    && (c->v == n->v)
                                    && (c->t >= n->t); });
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
                                     [a, t, m, n](Conflict* c) {
                                       if (c->a != a) return false;
                                       if (c->t != t) return false;
                                       if (c->onNode) return c->v == m;
                                       return c->v == m && c->u == n->v;
                                     });
      if (constraint != constraints.end()) continue;

      key = getKey(t, m);
      f = t + pathDist(m, g);
      auto itr2 = table.find(key);
      if (itr2 == table.end()) {
        l = new AN { m, true, t, f, n };
        table.emplace(key, l);
        tmpPath = getPartialPath(l);
        table_conflict.emplace(key, h3(a, tmpPath, paths));
      } else {
        l = table.at(key);
        if (!l->open) continue;
      }

      if (l->f > f) {
        l->f = f;
        l->p = n;
        tmpPath = getPartialPath(l);
        table_conflict.emplace(key, h3(a, tmpPath, paths));
      }

      tmpPath.clear();
      OPEN.insert(key);
    }
  }

  // back tracking
  int fmin = 0;
  if (!invalid) {  // check failed or not
    path = getPartialPath(n);
    fmin = table.at(*itrO)->f;
  } else {
    node->valid = false;
  }
  table_fmin.at(a->getId()) = fmin;

  return path;
}

struct ANF {
  Node* v;
  bool open;
  int t;
  float f;
  ANF* p;
};
int iECBS::highwayCost(Node* s, Node* g) {
  if (s == g) return 0;

  int s_index = G->getNodeIndex(s);
  int g_index = G->getNodeIndex(g);
  int dist = wdists(s_index, g_index);
  if (dist != 0) return dist;

  std::unordered_map<int, ANF*> table;
  ANF* t = new ANF { s, true, 0, static_cast<float>(pathDist(s, g)), nullptr };
  table.emplace(s->getId(), t);
  std::unordered_set<int> OPEN = { s->getId() };

  struct ANF* n = nullptr;
  struct ANF* l = nullptr;
  int id, f, d, cost;
  Nodes C;

  while (!OPEN.empty()) {
    auto itrO = std::min_element(OPEN.begin(), OPEN.end(),
                                 [&table] (int a, int b)
                                 { auto eleA = table.at(a);
                                   auto eleB = table.at(b);
                                   if (eleA->f == eleB->f) {
                                     return eleA->t > eleB->t;
                                   }
                                   return eleA->f < eleB->f; });
    id = *itrO;
    n = table.at(id);

    // check goal condition
    if (n->v == g) break;

    // already known?
    d = wdists(n->v->getIndex(), g_index);
    if (d != 0) {
      d = n->t + d;
      while (n->p) {
        n = n->p;
        wdists(n->v->getIndex(), g_index) = d - n->t;
      }
      return d;
    }

    // update list
    n->open = false;
    OPEN.erase(itrO);

    // search neighbor
    C = G->neighbor(n->v);

    for (auto m : C) {
      id = m->getId();
      auto itrT = table.find(id);
      if (itrT == table.end()) {
        ANF* t = new ANF { m, true, 0, 100000, nullptr };
        table.emplace(id, t);
      }
      l = table.at(id);
      if (!l->open) continue;
      cost = highway.at(std::to_string(n->v->getId()) + "-" + std::to_string(id));

      d = wdists(m->getIndex(), g_index);
      if (d != 0) {
        f = n->t + cost + d;
      } else {
        f = n->t + cost + pathDist(m, g);
      }

      if (l->f > f) {
        l->t = n->t + cost;
        l->f = f;
        l->p = n;
      }
      OPEN.insert(id);
    }
  }

  if (n->v != g) {
    std::cout << "error@iEPIBT::highwayCost, " << "cannot find a path : "
              << s->getId() << " -> " << g->getId() << "\n";
    std::exit(1);
  }

  d = n->t;
  wdists(s_index, g_index) = d;
  while (n->p) {
    n = n->p;
    wdists(n->v->getIndex(), g_index) = d - n->t;
  }

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
