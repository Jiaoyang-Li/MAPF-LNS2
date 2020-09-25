/*
 * winpibt.h
 *
 * Purpose: windowed PIBT
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#pragma once
#include "solver.h"


class winPIBT : public Solver {
protected:
  int w;  // window size
  bool softmode;
  Paths PATHS;
  std::vector<int> L;
  std::vector<float> epsilon;
  std::vector<int> eta;
  std::vector<float> priority;

  int ell(PIBT_Agent* a);
  int ell(int i);
  Node* lastNode(PIBT_Agent* a);
  Node* lastNode(int id);
  void updateGoal(PIBT_Agent* a);
  Node* getGoal(PIBT_Agent* a);
  PIBT_Agents::iterator findPITargetAgent(Node* v, int t);
  int getTmax(int t_tmp);

  void init();
  void allocate();

  virtual bool winpibt(PIBT_Agent* a, int t_tmp, bool varphi);

  Nodes getPath(PIBT_Agent* a, Node* g, int t1, int t2);
  bool checkValidPath(int id, Nodes &path, int t1, int t2);

  void updatePriority();

public:
  winPIBT(Problem* _P, int _w, bool _softmode);
  winPIBT(Problem* _P, int _w, bool _softmode, std::mt19937* _MT);
  ~winPIBT();

  virtual bool solve();

  virtual std::string logStr();
};
