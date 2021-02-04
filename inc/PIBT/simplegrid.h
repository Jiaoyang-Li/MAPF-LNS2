/*
 * simplegrid.h
 *
 * Purpose: simple graph, grid
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

/*
 * graph representation
 *
 * a : up
 * b : right
 * c : down
 * d : left
 * e : up & right
 * f : up & down
 * g : up & left
 * h : right & down
 * i : right & left
 * j : down & left
 * k : up & right & down
 * l : right & down & left
 * m : down & left & up
 * n : left & up & right
 * o : all
 * . : all
 * @ : object
 * T : object
 */


#pragma once
#include "grid.h"

class SimpleGrid : public Grid {
protected:
  std::string filename;

  void init();
  void setBasicParams();
  void createNodes();
  void createEdges();
  virtual void setStartGoal();

public:
  SimpleGrid(std::string _filename);
  SimpleGrid(std::string _filename, std::mt19937* _MT);
  ~SimpleGrid();

  // for iterative MAPF
  virtual Node* getNewGoal(Node* v);

  std::string getMapName() { return filename; }

  std::string logStr();
};
