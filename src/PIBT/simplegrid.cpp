/*
 * simplegrid.cpp
 *
 * Purpose: simple graph, grid
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include <fstream>
#include <regex>
#include "util.h"
#include "simplegrid.h"

SimpleGrid::SimpleGrid(Instance instance){
    setSize(instance.num_of_cols,instance.num_of_rows);

    //set nodes
    int id;
    for (int row=0;row<this->h;row++){
        for (int col=0;col<this->w;col++){
            id = row*this->w + col;
            if (instance.isObstacle(id))
                continue;
            Node* v = new Node(id);
            v->setPos(row, col);
            nodes.push_back(v);
        }
    }

    //set edge

    Nodes neighbor;
    Node* v;

    for (int row=0;row<this->h;row++){
        for (int col=0;col<this->w;col++){
            id = row*this->w + col;
            if (instance.isObstacle(id))
                continue;

            v = getNode(id);  // target node
            neighbor.clear();

            if (existNode(id - w)) neighbor.push_back(getNode(id - w));

            if (existNode(id + w)) neighbor.push_back(getNode(id + w));

            if (existNode(id - 1)) neighbor.push_back(getNode(id - 1));

            if (existNode(id + 1)) neighbor.push_back(getNode(id + 1));

            v->setNeighbor(neighbor);
        }
    }

    setStartGoal();
}

SimpleGrid::~SimpleGrid() {}


void SimpleGrid::setStartGoal() {
  // all nodes are target
  starts = nodes;
  goals = nodes;
}

Node* SimpleGrid::getNewGoal(Node* v) {
  Node* u;
  do {
    u = randomChoose(goals, MT);
  } while (u == v);
  return u;
}

std::string SimpleGrid::logStr() {
  std::string str = Grid::logStr();
  str += "[graph] filename:" + filename + "\n";
  return str;
}
