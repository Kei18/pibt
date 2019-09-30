/*
 * imapf_fair.cpp
 *
 * Purpose: considering fairness
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "imapf_fair.h"
#include "../util/util.h"

IMAPF_FAIR::IMAPF_FAIR(Graph* _G, Agents _A, int _taskLimit)
  : Problem(_G, _A), taskLimit(_taskLimit)
{
  init();
}

IMAPF_FAIR::IMAPF_FAIR(Graph* _G, Agents _A, int _taskLimit, std::mt19937* _MT)
  : Problem(_G, _A, _MT), taskLimit(_taskLimit)
{
  init();
}

IMAPF_FAIR::~IMAPF_FAIR() {}

void IMAPF_FAIR::init() {
  taskSum = 0;

  // initial allocation
  for (auto a : A) {
    goalCounts.push_back(0);
    allocate(a);
    a->updateHist();
  }
}

void IMAPF_FAIR::update() {
  ++timestep;
  Agent* a;

  for (int i = 0; i < A.size(); ++i) {
    a = A[i];
    auto tau = a->getTask();
    if (tau) {
      tau->update(a->getNode());

      if (tau->completed()) {
        ++taskSum;
        goalCounts[i] += 1;
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
}

bool IMAPF_FAIR::isSolved() {
  if (taskSum < taskLimit * A.size()) return false;
  // check all tasks are completed or not
  auto itr = std::find_if(goalCounts.begin(), goalCounts.end(),
                          [&] (int i) { return i < taskLimit; });
  if (itr != goalCounts.end()) return false;
  return true;
}

void IMAPF_FAIR::allocate(Agent* a) {
  Node* v = G->getNewGoal(a->getNode());
  Task* tau = new Task(v, timestep);
  a->setTask(tau);
  a->setGoal(a->getTask()->getG()[0]);
  T_OPEN.push_back(tau);
}

std::string IMAPF_FAIR::logStr() {
  std::string str = Problem::logStr();
  str += "[problem] type:IMAPF_FAIR\n";
  str += "[problem] agentnum:" + std::to_string(A.size()) + "\n";
  str += "[problem] tasknum:" + std::to_string(taskLimit) + "\n";
  str += G->logStr();
  for (auto tau : T_CLOSE) str += tau->logStr();
  for (auto a : A) str += a->logStr();
  return str;
}
