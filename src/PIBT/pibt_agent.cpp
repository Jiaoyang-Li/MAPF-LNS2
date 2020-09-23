/*
 * agent.cpp
 *
 * Purpose: Agent
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "pibt_agent.h"

int PIBT_Agent::cntId = 0;

PIBT_Agent::PIBT_Agent() : id(cntId) {
  ++cntId;
  g = nullptr;
  tau = nullptr;
  updated = false;
}

PIBT_Agent::PIBT_Agent(Node* _v) : id(cntId) {
  ++cntId;
  g = nullptr;
  tau = nullptr;
  setNode(_v);
  updated = false;
}

PIBT_Agent::~PIBT_Agent() {
  for (auto s : hist) delete s;
  hist.clear();
}

void PIBT_Agent::setNode(Node* _v) {
  beforeNode = v;
  v = _v;
}

void PIBT_Agent::updateHist() {
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

void PIBT_Agent::releaseTask() {
  g = nullptr;
  tau = nullptr;
}

void PIBT_Agent::releaseTaskOnly() {
  tau = nullptr;
}

void PIBT_Agent::releaseGoalOnly() {
  g = nullptr;
}

std::string PIBT_Agent::logStr() {
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
