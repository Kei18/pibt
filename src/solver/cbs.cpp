/*
 * cbs.cpp
 *
 * Purpose: Conflict-based Search
 *
 * Sharon, G., Stern, R., Felner, A., & Sturtevant, N. R. (2015).
 * Conflict-based search for optimal multi-agent pathfinding.
 *
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "cbs.h"
#include <numeric>
#include "../util/util.h"


CBS::CBS(Problem* _P) : Solver(_P), ID(false) {
  init();
}
CBS::CBS(Problem* _P, bool _ID) : Solver(_P), ID(_ID) {
  init();
}

CBS::~CBS() {}

void CBS::init() {
  G->setRegFlg(true);
}

bool CBS::solve() {
  solveStart();

  Paths PATHS;
  std::vector<Block> WHOLE;

  // initialize
  for (int i = 0; i < A.size(); ++i) PATHS.push_back({});

  if (!ID) {  // normal CBS
    solvePart(PATHS, A);
    formalizePath(PATHS);

  } else {  // with ID framework
    for (auto a : A) WHOLE.push_back({ a });

    // solve with independent detection
    while (true) {
      for (auto BLOCK : WHOLE) {
        if (!solvePart(PATHS, BLOCK)) {
          solveEnd();
          return false;  // failed
        }
      }
      formalizePath(PATHS);
      if (checkBlocks(PATHS, WHOLE)) break;

      // initialize
      PATHS.clear();
      for (int i = 0; i < A.size(); ++i) PATHS.push_back({});
    }
  }

  // register path
  int maxLength = getMaxLengthPaths(PATHS);
  for (int t = 1; t < maxLength; ++t) {
    for (int i = 0; i < A.size(); ++i) A[i]->setNode(PATHS[i][t]);
    P->update();
  }

  solveEnd();
  return true;
}

// for independent detection
bool CBS::checkBlocks(Paths& paths, std::vector<Block>& whole) {
  int maxLength = getMaxLengthPaths(paths);
  std::vector<Block> OPEN = whole;
  std::vector<Block> CLOSE, TMP;
  Block block1, block2;
  int i, j;
  bool checkAll = true;
  bool checkPart, checkFrac;
  whole.clear();

  while (!OPEN.empty()) {
    block1 = OPEN[0];
    openToClose(block1, OPEN, CLOSE);

    checkFrac = true;
    for (auto block2 : OPEN) {

      checkPart = true;
      for (auto a : block1) {
        if (!checkPart) break;

        auto itrA = std::find_if(A.begin(), A.end(),
                                 [a](Agent* _a) { return a == _a; });
        i = std::distance(A.begin(), itrA);

        for (auto b : block2) {
          if (!checkPart) break;

          auto itrB = std::find_if(A.begin(), A.end(),
                                   [b](Agent* _b) { return b == _b; });
          j = std::distance(A.begin(), itrB);

          for (int t = 0; t < maxLength; ++t) {
            // detect collision
            if (paths[j][t] == paths[i][t]) {
              checkPart = false;
              break;
            }
            // intersection
            if (t == 0) continue;
            if (paths[j][t] == paths[i][t-1] && paths[j][t-1] == paths[i][t]) {
              checkPart = false;
              break;
            }
          }
        }
      }

      // merge
      if (!checkPart) {
        TMP.push_back(block2);
        checkFrac = false;
      }
    }

    if (!checkFrac) checkAll = false;
    Block newBlock;
    newBlock.insert(newBlock.end(), block1.begin(), block1.end());
    for (auto block2 : TMP) {
      openToClose(block2, OPEN, CLOSE);
      newBlock.insert(newBlock.end(), block2.begin(), block2.end());
    }
    TMP.clear();
    whole.push_back(newBlock);
  }

  OPEN.clear();
  CLOSE.clear();
  return checkAll;
}

// main
bool CBS::solvePart(Paths& paths, Block& block) {
  CTNode* node;
  Constraints constraints;
  std::vector<CTNode*> OPEN;
  bool status = true;

  CTNode* root = new CTNode { {}, {}, 0, nullptr, true, {}, 0 };
  invoke(root, block);
  OPEN.push_back(root);
  bool CAT = std::any_of(paths.begin(), paths.end(),
                         [](std::vector<Node*> p) { return !p.empty(); });

  bool invalid = true;
  while (!OPEN.empty()) {
    auto itr = std::min_element(OPEN.begin(), OPEN.end(),
                                [CAT, this, &paths]
                                (CTNode* n1, CTNode* n2) {
                                  if (CAT && n1->cost == n2->cost) {
                                    return this->countCollisions(n1, paths)
                                      < this->countCollisions(n2, paths);
                                  }
                                  return n1->cost < n2->cost;
                                });
    node = *itr;
    constraints = valid(node, block);
    if (constraints.empty()) {
      invalid = false;
      break;
    }
    OPEN.erase(itr);

    for (auto constraint : constraints) {
      CTNode* newNode = new CTNode { constraint, node->paths,
                                     0, node, true,
                                     {}, 0 };
      invoke(newNode, block);
      if (newNode->valid) OPEN.push_back(newNode);
    }
    constraints.clear();
  }

  if (!invalid) {  // sucssess
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

int CBS::countCollisions(CTNode* c, Paths& paths) {
  int collision = 0;
  for (auto p1 : paths) {
    if (p1.empty()) continue;
    for (auto p2 : c->paths) {
      if (p2.empty()) continue;
      for (int t = 0; t < p1.size(); ++t) {
        if (p2.size() < t) {
          if (p1[t] == p2[p2.size() - 1]) ++collision;
          continue;
        }
        if (p1[t] == p2[t]) {
          ++collision;
          continue;
        }
        if (t > 0 && p1[t-1] == p2[t] && p1[t] == p2[t-2]) {
          ++collision;
          continue;
        }
      }
    }
  }

  return collision;
}

void CBS::invoke(CTNode* node, Block& block) {
  // calc path
  if (node->c.empty()) {  // initial
    Paths paths;
    for (int i = 0; i < A.size(); ++i) paths.push_back({});
    for (auto a : block) {
      auto itr = std::find_if(A.begin(), A.end(),
                              [a](Agent* b) { return a == b; });
      paths[std::distance(A.begin(), itr)] = AstarSearch(a, node);
    }
    node->paths = paths;
  } else {
    Agent* a;
    for (auto c : node->c) {
      a = c->a;
      auto itr = std::find_if(A.begin(), A.end(),
                              [a](Agent* b) { return a == b; });
      node->paths[std::distance(A.begin(), itr)].clear();
    }
    for (auto c : node->c) {
      a = c->a;
      auto itr = std::find_if(A.begin(), A.end(),
                              [a](Agent* b) { return a == b; });
      node->paths[std::distance(A.begin(), itr)] = AstarSearch(a, node);
    }
  }
  if (!node->valid) return;
  calcCost(node, block);
  formalizePathBlock(node, block);
}

void CBS::calcCost(CTNode* node, Block& block) {
  int cost = 0;
  Node* g;
  std::vector<Node*> path;
  for (auto a : block) {
    auto itr1 = std::find_if(A.begin(), A.end(),
                            [a](Agent* b) { return a == b; });
    g = a->getGoal();
    path = node->paths[std::distance(A.begin(), itr1)];
    auto itr2 = path.end() - 1;
    while (g == *itr2) --itr2;
    cost += std::distance(path.begin(), itr2);
  }
  node->cost = cost;
}

Constraints CBS::valid(CTNode* node, Block& block) {
  Paths paths = node->paths;
  int maxLength = getMaxLengthPaths(node->paths);
  Constraints constraints;
  std::vector<int> ids;
  Node* v;
  Agent *a, *b;
  int i, j;

  for (int t = 0; t < maxLength; ++t) {
    for (int _i = 0; _i < block.size(); ++_i) {
      a = block[_i];
      auto itr1 = std::find_if(A.begin(), A.end(),
                               [a](Agent* _a)
                               { return a->getId() == _a->getId(); });
      i = std::distance(A.begin(), itr1);
      v = paths[i][t];

      // check collision
      ids.clear();
      for (int _j = _i + 1; _j < block.size(); ++_j) {
        b = block[_j];
        auto itr2 = std::find_if(A.begin(), A.end(),
                                 [b](Agent* _b) { return b == _b; });
        j = std::distance(A.begin(), itr2);
        if (paths[j][t] == v) ids.push_back(j);
      }

      if (!ids.empty()) {  // detect collision
        ids.push_back(i);
        for (int k = 0; k < ids.size(); ++k) {
          Constraint constraint;
          for (int l = 0; l < ids.size(); ++l) {
            if (k == l) continue;
            Conflict* c = new Conflict { A[ids[l]], t, v };
            constraint.push_back(c);
          }
          constraints.push_back(constraint);
        }
        return constraints;
      }

      // check intersection
      if (t == 0) continue;
      auto itr3 = paths.begin() + i;
      auto itr4 = std::find_if(itr3 + 1, paths.end(),
                               [t, itr3](std::vector<Node*> path) {
                                 if (path.empty()) return false;
                                 return (path[t] == (*itr3)[t-1])
                                   && (path[t-1] == (*itr3)[t]); });
      if (itr4 != paths.end()) {  // detect intersection

        Constraint knownConstraint = getConstraitns(node);

        j = std::distance(paths.begin(), itr4);
        Conflict* c1 = new Conflict { A[i], t, (*itr3)[t] };
        Conflict* c2 = new Conflict { A[j], t, (*itr4)[t] };

        // checking duplication
        if (!isDuplicatedConflict(knownConstraint, c1)) constraints.push_back({ c1 });
        if (!isDuplicatedConflict(knownConstraint, c2)) constraints.push_back({ c2 });

        if (t == 1) return constraints;
        Conflict* c3 = new Conflict { A[i], t-1, (*itr3)[t-1] };
        Conflict* c4 = new Conflict { A[j], t-1, (*itr4)[t-1] };
        if (!isDuplicatedConflict(knownConstraint, c3)) constraints.push_back({ c3 });
        if (!isDuplicatedConflict(knownConstraint, c4)) constraints.push_back({ c4 });
        return constraints;
      }
    }
  }

  return constraints;
}

void CBS::formalizePathBlock(CTNode* node, Block& block) {
  int maxLength = getMaxLengthPaths(node->paths);
  Node* g;
  int i;
  for (auto a : block) {
    auto itr = std::find_if(A.begin(), A.end(),
                            [a](Agent* b) { return a == b; });
    i = std::distance(A.begin(), itr);
    g = node->paths[i][node->paths[i].size() - 1];
    while (node->paths[i].size() != maxLength) node->paths[i].push_back(g);
  }
}

// low-level search
std::vector<Node*> CBS::AstarSearch(Agent* a, CTNode* node) {
  std::vector<Conflict*> constraints = getConstraitnsForAgent(node, a);

  std::vector<Node*> path;  // return
  Node* s = a->getNode();
  Node* g = a->getGoal();

  if (constraints.empty()) return G->getPath(s, g);

  int t = 0;
  std::vector<Node*> C;  // candidates
  int f = 0;
  std::string key;
  AN *l, *n;
  std::unordered_set<std::string> OPEN;
  std::unordered_map<std::string, AN*> table;

  l = new AN { s, true, 0, 0, nullptr };
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

    // check goal
    if (n->v == g) {
      auto check = std::find_if(constraints.begin(), constraints.end(),
                                [n] (Conflict* c) {
                                  return (c->v == n->v) && (c->t >= n->t); });
      if (check == constraints.end()) {
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
      // check constraints
      t = n->t + 1;
      auto constraint = std::find_if(constraints.begin(), constraints.end(),
                                     [a, t, m](Conflict* c) {
                                       return (c->a == a)
                                         && (c->t == t) && (c->v == m); });
      if (constraint != constraints.end()) continue;

      key = getKey(t, m);
      auto itr2 = table.find(key);
      if (itr2 == table.end()) {
        table.emplace(key, new AN { m, true, t, 1000000, n });
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
  } else {
    node->valid = false;
  }

  return path;
}

std::vector<Conflict*> CBS::getConstraitns(CTNode* ctNode) {
  std::vector<Conflict*> constraints;
  if (ctNode->p == nullptr) return constraints;  // root
  while (ctNode->p != nullptr) {
    for (auto c : ctNode->c) constraints.push_back(c);
    ctNode = ctNode->p;
  }
  return constraints;
}

std::vector<Conflict*> CBS::getConstraitnsForAgent(CTNode* ctNode, Agent* a) {
  std::vector<Conflict*> constraints;
  if (ctNode->p == nullptr) return constraints;  // root
  while (ctNode->p != nullptr) {
    for (auto c : ctNode->c) {
      if (c->a == a) constraints.push_back(c);
    }
    ctNode = ctNode->p;
  }
  return constraints;
}

bool CBS::isDuplicatedConflict(Constraint &constraint, Conflict* c) {
  auto itr = std::find_if(constraint.begin(), constraint.end(),
                          [c] (Conflict* d)
                                { return c->a == d->a
                                    && c->t == d->t
                                    && c->v == d->v ; });
  return itr != constraint.end();
}

std::string CBS::logStr() {
  std::string str;
  str += "[solver] type:CBS\n";
  str += Solver::logStr();
  return str;
}
