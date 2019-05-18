/*
 * imapf.cpp
 *
 * Purpose: Iterative MAPF
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "imapf.h"
#include "../util/util.h"


IMAPF::IMAPF(Graph* _G, Agents _A, int _taskLimit)
  : Problem(_G, _A), taskLimit(_taskLimit)
{
  init();
}

IMAPF::IMAPF(Graph* _G, Agents _A, int _taskLimit, std::mt19937* _MT)
  : Problem(_G, _A, _MT), taskLimit(_taskLimit)
{
  init();
}

IMAPF::~IMAPF() {}

void IMAPF::init() {
  taskSum = 0;
  taskSumStr = "0,";
  nodes = G->getNodes();

  // initial allocation
  for (auto a : A) {
    allocate(a);
    a->updateHist();
  }
}

bool IMAPF::isSolved() {
  if (taskSum < taskLimit) return false;
  // check all tasks are completed or not
  for (int i = 0; i < taskLimit; ++i) {
    auto itr = std::find_if(T_CLOSE.begin(), T_CLOSE.end(),
                            [i] (Task* tau) { return tau->getId() == i; });
    if (itr == T_CLOSE.end()) return false;
  }
  return true;;
}

void IMAPF::update() {
  ++timestep;

  for (auto a : A) {
    auto tau = a->getTask();
    if (tau) {
      tau->update(a->getNode());

      if (tau->completed()) {
        ++taskSum;
        tau->setEndTime(timestep);
        a->releaseTask();
        openToClose(tau, T_OPEN, T_CLOSE);
        allocate(a);
        a->goalUpdated(true);
      } else {
        a->goalUpdated(false);
      }
    } else {
      allocate(a);
    }
    a->updateHist();
  }

  taskSumStr += std::to_string(taskSum) + ",";
}

void IMAPF::allocate(Agent* a) {
  Node* v = G->getNewGoal(a->getNode());
  Task* tau = new Task(v, timestep);
  a->setTask(tau);
  a->setGoal(a->getTask()->getG()[0]);
  T_OPEN.push_back(tau);
}

std::string IMAPF::logStr() {
  std::string str = Problem::logStr();
  str += "[problem] type:IMAPF\n";
  str += "[problem] agentnum:" + std::to_string(A.size()) + "\n";
  str += "[problem] tasknum:" + std::to_string(taskLimit) + "\n";
  str += "[problem] tasksum:" + taskSumStr + "\n";
  str += G->logStr();
  for (auto tau : T_CLOSE) str += tau->logStr();
  for (auto a : A) str += a->logStr();
  return str;
}
