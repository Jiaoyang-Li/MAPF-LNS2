/*
 * Grid.cpp
 *
 * Purpose: Grid
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "grid.h"


Grid::Grid(std::mt19937* _MT) : Graph(_MT) {}
Grid::Grid() {};
Grid::~Grid() {}

void Grid::setSize(int _w, int _h) {
  w = _w;
  h = _h;
}

int Grid::manhattanDist(Node* v, Node* u) {
  Vec2f vPos = v->getPos();
  Vec2f uPos = u->getPos();
  return (int)(std::abs((int)(vPos.x - uPos.x))
               + std::abs((int)(vPos.y - uPos.y)));
}

Nodes Grid::getPath(Node* s, Node* g) {
  Nodes nodes = {};
  return Graph::getPath(s, g, nodes, manhattanDist);
}

Nodes Grid::getPath(Node* s, Node* g, Nodes &prohibitedNodes) {
  return Graph::getPath(s, g, prohibitedNodes, manhattanDist);
}

Nodes Grid::getPath(int s, int g) {
  return getPath(getNode(s), getNode(g));
}

std::string Grid::logStr() {
  std::string str;
  str += "[graph] width:" + std::to_string(getW()) + "\n";
  str += "[graph] height:" + std::to_string(getH()) + "\n";
  return str;
}
