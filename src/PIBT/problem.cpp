/*
 * problem.cpp
 *
 * Purpose: Definition of Problem
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "problem.h"
#include "util.h"
#include <random>


Problem::Problem(Graph* _G,
                 PIBT_Agents _A,
                 std::vector<Task*> _T) : G(_G), A(_A), T_OPEN(_T){
  std::random_device seed_gen;
  MT = new std::mt19937(seed_gen());
  init();
}

Problem::Problem(Graph* _G, PIBT_Agents _A) : G(_G), A(_A) {
  std::random_device seed_gen;
  MT = new std::mt19937(seed_gen());
  init();
}

Problem::Problem(Graph* _G,
                 PIBT_Agents _A,
                 std::vector<Task*> _T,
                 std::mt19937* _MT) : G(_G), A(_A), T_OPEN(_T), MT(_MT) {
  init();
}

Problem::Problem(Graph* _G,
                 PIBT_Agents _A,
                 std::mt19937* _MT) : G(_G), A(_A), MT(_MT) {
  init();
}

void Problem::init() {
  timestep = 0;
}

Problem::~Problem() {
  for (auto a : A) delete a;
  for (auto t : T_OPEN) delete t;
  for (auto t : T_CLOSE) delete t;
  A.clear();
  T_OPEN.clear();
}

void Problem::assign(Task* tau) {
  openToClose(tau, T_OPEN, T_CLOSE);
}

std::string Problem::logStr() {
  return "[problem] timesteplimit:" + std::to_string(timesteplimit) + "\n";
}
