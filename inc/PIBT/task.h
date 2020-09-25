#pragma once

#include <vector>
#include "node.h"


class Task {
private:
  std::vector<Node*> G_OPEN;   // remained nodes
  std::vector<Node*> G_CLOSE;  // finished nodes

  const int id;
  static int cntId;  // for uuid

  int startTime;  // timestep
  int endTime;

public:
  Task();
  Task(Node* v);
  Task(std::vector<Node*> nodes);
  Task(int t);
  Task(Node* v, int t);
  ~Task();

  int getId() { return id; }

  std::vector<Node*> getG() { return G_OPEN; }
  void setG(std::vector<Node*> g) { G_OPEN = g; }

  void addNode(Node* v) { G_OPEN.push_back(v); }

  Node* getNext(Node* v);

  void update(Node* v);

  bool completed();
  void setEndTime(int t);

  std::string logStr();
};
