/*
 * task.cpp
 *
 * Purpose: Task
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "task.h"
#include "util.h"


int Task::cntId = 0;


Task::Task() : id(cntId) {
  ++cntId;
  startTime = 0;
  endTime = 0;
}

Task::Task(Node* v) : id(cntId) {
  ++cntId;
  startTime = 0;
  endTime = 0;
  G_OPEN.push_back(v);
}

Task::Task(int t) : id(cntId) {  // for mapd
  ++cntId;
  startTime = t;
  endTime = 0;
}

Task::Task(Node* v, int t) : id(cntId) {
  ++cntId;
  startTime = t;
  endTime = 0;
  G_OPEN.push_back(v);
}

Task::Task(std::vector<Node*> nodes) : id(cntId) {
  ++cntId;
  for (auto v : nodes) G_OPEN.push_back(v);
}

Task::~Task() {
  G_OPEN.clear();
  G_CLOSE.clear();
}

void Task::update(Node* g) {
  openToClose(g, G_OPEN, G_CLOSE);
}

bool Task::completed() {
  return G_OPEN.empty();
}

void Task::setEndTime(int t) {
  endTime = t;
}

Node* Task::getNext(Node* g) {
  auto itr = G_OPEN.begin();
  while (*itr != g || itr != G_OPEN.end()) ++itr;
  if (itr == G_OPEN.end()) return nullptr;
  ++itr;
  if (itr == G_OPEN.end()) return nullptr;
  return *itr;
}

std::string Task::logStr() {
  std::string str;
  str += "[task] ";
  str += "id:" + std::to_string(id);
  str += ",start:" + std::to_string(startTime);
  str += ",end:" + std::to_string(endTime);
  str += ",service time:" + std::to_string(endTime - startTime);
  str += ",nodes:";
  for (auto v : G_CLOSE) str += std::to_string(v->getId()) + ",";
  str += "\n";
  return str;
}
