#pragma once

#include <iostream>

#include "graph.h"


class Grid : public Graph {
protected:
  int w;
  int h;
  void setSize(int _w, int _h);

public:
  Grid();
  Grid(std::mt19937* _MT);
  ~Grid();

  int getW() { return w; }
  int getH() { return h; }

  static int manhattanDist(Node* v, Node* u);

  Nodes getPath(Node* s, Node* g);
  Nodes getPath(Node* s, Node* g, Nodes &prohibitedNodes);
  Nodes getPath(int s, int g);

  int dist(Node* v1, Node* v2) { return manhattanDist(v1, v2); }

  virtual std::string logStr();
};
