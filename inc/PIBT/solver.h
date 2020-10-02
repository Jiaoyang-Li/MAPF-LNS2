#pragma once

#include "problem.h"
#include <vector>
#include <algorithm>
#include <chrono>
#include <eigen3/Eigen/Core>
#include <unordered_map>
#include <unordered_set>
#include <boost/heap/fibonacci_heap.hpp>

typedef std::chrono::duration<float> fsec;



class Solver {
private:
  double elapsedTime;

protected:
  Problem* P;
  std::mt19937* MT;

  PIBT_Agents A;
  Graph* G;

  Eigen::MatrixXi dists;

  void init();
  int getMaxLengthPaths(Paths& paths);
  void formalizePath(Paths& paths);
  int pathDist(Node* v, Node* u);
  int pathDist(Node* s, Node* g, Nodes &prohibited);
  std::vector<PIBT_Agents> findAgentBlock();
  static std::string getKey(int t, Node* v);
  static std::string getKey(AN* n);

  virtual void solveStart();
  virtual void solveEnd();
  std::chrono::system_clock::time_point startT;
  std::chrono::system_clock::time_point endT;
  double time_limit=0;

public:
  Solver(Problem* _P);
  Solver(Problem* _P, std::mt19937* _MT);
  ~Solver();

  void WarshallFloyd();
  void setTimeLimit(double limit){this->time_limit=limit;};


    virtual bool solve() { return false; };
  double getElapsed() { return elapsedTime; };

  virtual std::string logStr();
};
