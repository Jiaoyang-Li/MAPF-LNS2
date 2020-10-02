/*
 * mapf.cpp
 *
 * Purpose: MAPF
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "mapf.h"
#include "util.h"

MAPF::MAPF(Graph* _G,
           PIBT_Agents _A,
           std::vector<Task*> _T) : Problem(_G, _A, _T)
{
  init();
}

MAPF::MAPF(Graph* _G,
           PIBT_Agents _A,
           std::vector<Task*> _T,
           std::mt19937* _MT) : Problem(_G, _A, _T, _MT)
{
  init();
}

void MAPF::init() {
  // error check
  if (A.size() != T_OPEN.size()) {
    std::cout << "error@MAPF::init, this is not a MAPF, |A| != |T|" << "\n";
    std::exit(1);
  }

  // allocation, t=0
  for (int i = 0; i < A.size(); ++i) {
    A[i]->setTask(T_OPEN[i]);
    A[i]->setGoal(A[i]->getTask()->getG()[0]);
    A[i]->updateHist();
  }
}

MAPF::~MAPF() {}

bool MAPF::isSolved() {
  if (!T_OPEN.empty()) return false;
  return std::all_of(A.begin(), A.end(), [] (PIBT_Agent* a) { return a->getGoal() == a->getNode(); });
}

void MAPF::update() {
  ++timestep;

  for (auto a : A) {
    auto tau = a->getTask();
    if (tau) {  // if agent has a task
      tau->update(a->getNode());  // update task status
      if (tau->completed()) {  // if task is completed
        a->releaseTaskOnly();  // agent release task, not goal
        openToClose(tau, T_OPEN, T_CLOSE);
      }
    } else if (a->getNode() != a->getGoal()) {
      // create new task and assign
      Task* newTau = new Task(a->getGoal());
      a->setTask(newTau);
      a->setGoal(newTau->getG()[0]);
      T_OPEN.push_back(newTau);
    }
    a->updateHist();
  }
}

std::string MAPF::logStr() {
  std::string str = Problem::logStr();
  str += "[problem] type:MAPF\n";
  str += "[problem] agentnum:" + std::to_string(A.size()) + "\n";
  str += G->logStr();
  for (auto tau : T_CLOSE) str += tau->logStr();

  int cnt;
  int pathsize;
  for (auto a : A) {
    auto hist = a->getHist();
    auto itr = hist.end() - 1;
    cnt = 0;
    while ((*itr)->v == a->getGoal()) {
      ++cnt;
      --itr;
    }
    pathsize = getTerminationTime() - cnt + 1;
    str += a->logStr();
    str += "size:" + std::to_string(pathsize) + "\n";
  }
  return str;
}
