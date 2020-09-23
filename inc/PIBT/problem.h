#pragma once
#include "graph.h"
#include "pibt_agent.h"
#include "task.h"

using PIBT_Agents = std::vector<PIBT_Agent*>;
// defined in Graph.h
// using Nodes = std::vector<Node*>;
// using Paths = std::vector<Nodes>;

class Problem {
protected:
  int timestep;
  int timesteplimit;
  Graph *G;
  PIBT_Agents A;
  std::vector<Task*> T_OPEN;   // open tasks
  std::vector<Task*> T_CLOSE;  // close tasks

  std::mt19937* MT;

  void init();

public:
  Problem(Graph* _G, PIBT_Agents _A, std::vector<Task*> _T);
  Problem(Graph* _G, PIBT_Agents _A);
  Problem(Graph* _G, PIBT_Agents _A, std::vector<Task*> _T, std::mt19937* _MT);
  Problem(Graph* _G, PIBT_Agents _A, std::mt19937* _MT);

  virtual ~Problem();

  virtual bool isSolved() { return false; }
  virtual void update() {}
  virtual bool allocated() { return false; }  // whether tasks are assigned beforehand?

  Graph* getG() { return G; }
  PIBT_Agents getA() { return A; }
  std::vector<Task*> getT() { return T_OPEN; }
  int getTerminationTime() { return timestep; }
  int getTimestep() { return timestep; }

  int getTimestepLimit() { return timesteplimit; }
  void setTimestepLimit(int _t) { timesteplimit = _t; }

  void assign(Task* tau);
  virtual void setAutoAssignement(bool flg) {}

  // for visualization
  bool visual_showicon;
  std::string visual_icon;

  virtual std::string logStr();
};
