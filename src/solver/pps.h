/*
 * pps.h
 *
 * Purpose: Parallel Push & Swap
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#pragma once
#include "solver.h"


enum RES { SUCCESS,
           FAIL,
           PAUSE };
enum CHECK { VALID, INVALID };
enum SWAPPHASE { GO_TARGET,
                 CLEARING,
                 EVAC_H,
                 EVAC_L,
                 SWAP_DONE };

struct S {
  int id;
  Agents agents;  // priority : high priority, low priority
  Node* lowOriginalNode;  // [low] original pos, [high] goal pos
  Nodes esv;  // swap node candidates
  Node* target;  // swap node, deg(target) >= 3
  Node* origin;  // low pos when swap starts
  Node* evacH;  // [high] evacuation node
  Node* evacL;  // [low]  evacuation node
  Nodes area;   // swap area, using case 3
  SWAPPHASE phase;  // phase
};

class PPS : public Solver {
private:
  Agents pushers;  // pushers
  std::vector<S*> swapers;   // swapers
  Agents pusherToSwaper;  // tmp
  Agents swaperToPusher;  // tmp
  std::vector<S*> doneSwapers;  // tmp

  Agents M;  // moved agents
  Agents U;  // goal agents
  Nodes L;   // reserved nodes
  bool status;  // continue or not

  Nodes goals;  // final goal
  std::vector<bool> isTmpGoals;  // has temp goal
  Nodes deg3nodes;

  static int s_uuid;

  void init();

  RES SWAP(S* s);

  RES PUSH(Agent* c, bool swap);
  RES PUSH(Agent* c, Nodes &T, bool swap);
  RES PUSH(Agent* c, Agents &H, bool swap);
  RES PUSH(Agent* c, Agents &H, Nodes &T, bool swap);
  RES PUSH(S* s);

  RES FEASIBLE(Agent* c, Nodes &pi, Agents& H, Nodes& T, bool swap);
  RES FEASIBLE(S* s, Nodes &pi);

  Nodes SHORTEST_PATH(Node* s, Node* g);
  Nodes SHORTEST_PATH(Agent* c, Node* g);
  Nodes SHORTEST_PATH(Agent* c, Node* g, Agents& H);
  Nodes SHORTEST_PATH(Node* s, Node* g, Nodes prohibited);
  Nodes SHORTEST_PATH(Agent* c, Node* g, Agents& H, Nodes& T);

  bool DEPEND(Nodes piA, Nodes piB);

  void SETUP_SWAP(Agent* c, Agent* a);

  bool CLEAR(S* s);

  void SWAP_PRIMITIVES(S* s);
  void FINISH_SWAP(S* s);

  void move(Agent* a, Nodes &pi);
  void move(S* s, Nodes &pi);

  bool reserved(Node* v, Agents &M);
  bool isFree(Node* v);
  bool inS(Agent* a);
  S* getS(Agent* a);
  Nodes getSortedEsv(Agent* c);

  CHECK CHECK_PRIORITY(S* s, Agent* a);
  CHECK CHECK_PRIORITY(Agent* c, Agent* a);
  CHECK CHECK_PRIORITY(S* sC, S* sA);

  RES FIND_NEW_VERTEX(S* s);

  CHECK CHECK_PUSHER(Agent* a);
  CHECK CHECK_SWAPER(S* s);
  CHECK CHECK_GOAL(Agent* a);

  void ADD_DONE_SWAPERS(S* s);
  Nodes CLOSEST_EMPTY_VERTICLES(Agent* c);

public:
  PPS(Problem* _P);
  PPS(Problem* _P, std::mt19937* _MT);
  ~PPS();

  bool solve();
  void update();

  std::string logStr();
};
