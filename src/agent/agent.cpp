/*
 * agent.cpp
 *
 * Purpose: Agent
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "agent.h"
#include "../util/util.h"

int Agent::cntId = 0;

Agent::Agent() : id(cntId) {
  ++cntId;
  g = nullptr;
  tau = nullptr;
  updated = false;
  beforeNode = nullptr;
}

Agent::Agent(Node* _v) : id(cntId) {
  ++cntId;
  g = nullptr;
  tau = nullptr;
  v = nullptr;
  setNode(_v);
  updated = false;
}

Agent::~Agent() {
  for (auto s : hist) delete s;
  hist.clear();
}

void Agent::setNode(Node* _v) {
  // error check
  if (v != nullptr) {
    auto neigh = v->getNeighbor();
    if (!(_v == v || inArray(_v, neigh))) {
      std::cout << "error@Agent, set invalid node, from "
                << v->getId() << " to " << _v->getId() << std::endl;
      std::exit(1);
    }
  }

  beforeNode = v;
  v = _v;
}

void Agent::updateHist() {
  AgentStatus* s = new AgentStatus;
  s->v = v;
  if (hasGoal()) {
    s->g = g;
  } else {
    s->g = nullptr;
  }
  if (hasTask()) {
    s->tau = tau;
  } else {
    s->tau = nullptr;
  }
  hist.push_back(s);
}

void Agent::releaseTask() {
  g = nullptr;
  tau = nullptr;
}

void Agent::releaseTaskOnly() {
  tau = nullptr;
}

void Agent::releaseGoalOnly() {
  g = nullptr;
}

std::string Agent::logStr() {
  std::string str, strPath, strGoal;
  str += "[agent] ";
  str += "id:" + std::to_string(id) + "\n";
  strPath = "path:";
  strGoal = "goal:";
  for (auto s : hist) {
    strPath += std::to_string(s->v->getId()) + ",";
    if (s->g != nullptr) {
      strGoal += std::to_string(s->g->getId()) + ",";
    } else {
      strGoal += "*,";
    }
  }
  str += strPath + "\n";
  str += strGoal + "\n";
  return str;
}
