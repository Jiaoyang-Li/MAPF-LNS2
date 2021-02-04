/*
 * pibt.cpp
 *
 * Purpose: PIBT
 *
 * Okumura, K., Machida, M., Défago, X., & Tamura, Y. (2019).
 * Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding.
 * arxiv:1901.11282
 *
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "pibt.h"
#include <algorithm>
#include <random>
#include "util.h"


PIBT::PIBT(Problem* _P) : Solver(_P)
{
  init();
}

PIBT::PIBT(Problem* _P, std::mt19937* _MT) : Solver(_P, _MT)
{
  init();
}

PIBT::~PIBT() {}

void PIBT::init() {
  G->setRegFlg(true);

  // initialize priroirty
  int agentNum = A.size();
  for (int i = 0; i < agentNum ; ++i) {
    epsilon.push_back((float)i / agentNum);
    eta.push_back(0);
    priority.push_back(epsilon[i] + eta[i]);
  }
}


bool PIBT::solve() {
  solveStart();

  while (!P->isSolved()) {
    allocate();
    update();
    P->update();
      if(time_limit&&((fsec)(std::chrono::system_clock::now()-startT)).count()>time_limit){
          break;
      }
  }

  solveEnd();
  return P->isSolved();
}

void PIBT::allocate() {
  if (P->allocated()) return;
  auto T = P->getT();
  Graph* _G = G;

  for (auto a : A) {
    if (a->hasTask()) continue;
    if (T.empty()) {
      a->releaseGoalOnly();
    } else {
      auto v = a->getNode();
      auto itr = std::min_element(T.begin(), T.end(),
                                  [v, _G] (Task* t1, Task* t2) {
                                    return _G->dist(t1->getG()[0], v)
                                      < _G->dist(t2->getG()[0], v);
                                  });
      a->setGoal((*itr)->getG()[0]);
    }
  }
}

void PIBT::update() {
  updatePriority();

  std::vector<float> PL(priority.size());  // priority list
  std::copy(priority.begin(), priority.end(), PL.begin());

  Nodes CLOSE_NODE;
  PIBT_Agents OPEN_AGENT(A.size());
  std::copy(A.begin(), A.end(), OPEN_AGENT.begin());

  // choose one agent with the highest priority
  auto itr = std::max_element(PL.begin(), PL.end());
  int index = std::distance(PL.begin(), itr);
  PIBT_Agent* a = OPEN_AGENT[index];

  while (!OPEN_AGENT.empty()) {
    priorityInheritance(a, CLOSE_NODE, OPEN_AGENT, PL);

    itr = std::max_element(PL.begin(), PL.end());
    index = std::distance(PL.begin(), itr);
    a = OPEN_AGENT[index];
  }
}

void PIBT::updatePriority() {
  // update priority
  for (int i = 0; i < A.size(); ++i) {
    // hasTask & not reach to goal
    if (A[i]->isUpdated()) {
      eta[i] = 0;
    } else if (A[i]->hasTask() && (A[i]->getNode() != A[i]->getGoal())) {
      eta[i] += 1;
    } else {
      eta[i] = 0;
    }
    priority[i] = eta[i] + epsilon[i];
    // priority[i] = getDensity(A[i]);
  }
}

float PIBT::getDensity(PIBT_Agent* a) {
  float density = 0;
  Node *v, *u;
  int d, tmp;
  v = a->getNode();
  Nodes Ci, Cj;
  Ci = G->neighbor(v);

  for (auto b : A) {
    if (b == a) continue;
    u = b->getNode();
    d = G->dist(u, v);
    if (G->dist(u, v) > 2) continue;
    tmp = - d + 2;
    Cj = G->neighbor(u);
    for (auto w : Cj) {
      if (w == v) {
        tmp += 2;
      } else if (G->dist(w, u) == 1) {
        ++tmp;
      }
    }
    density += (float)tmp / (float)Cj.size();
  }

  density /= (float)Ci.size();
  return density;
}

bool PIBT::priorityInheritance(PIBT_Agent* a,
                               Nodes& CLOSE_NODE,
                               PIBT_Agents& OPEN_AGENT,
                               std::vector<float>& PL) {
  Nodes C = createCandidates(a, CLOSE_NODE);
  return priorityInheritance(a, C, CLOSE_NODE, OPEN_AGENT, PL);
}

bool PIBT::priorityInheritance(PIBT_Agent* a,
                               PIBT_Agent* aFrom,
                               Nodes& CLOSE_NODE,
                               PIBT_Agents& OPEN_AGENT,
                               std::vector<float>& PL) {
  Nodes C = createCandidates(a, CLOSE_NODE, aFrom->getNode());
  return priorityInheritance(a, C, CLOSE_NODE, OPEN_AGENT, PL);
}

bool PIBT::priorityInheritance(PIBT_Agent* a,
                               Nodes C,
                               Nodes& CLOSE_NODE,
                               PIBT_Agents& OPEN_AGENT,
                               std::vector<float>& PL)
{
  // remove agent from OPEN_AGENT, Priority List
  auto itr = std::find(OPEN_AGENT.begin(), OPEN_AGENT.end(), a);
  PL.erase(PL.begin() + std::distance(OPEN_AGENT.begin(), itr));
  OPEN_AGENT.erase(itr);

  Node* target;

  // main loop
  while (!C.empty()) {

    // choose target
    target = chooseNode(a, C);
    CLOSE_NODE.push_back(target);

    bool isAgent = false;
    for (auto b : OPEN_AGENT) {
      if (target == b->getNode()) {  // If there is an agent
        if (priorityInheritance(b, a, CLOSE_NODE, OPEN_AGENT, PL)) {
          // priority inheritance success
          a->setNode(target);
          return true;
        } else {
          // priority inheritance fail
          updateC(C, target, CLOSE_NODE);
          isAgent = true;
          break;
        }
      }
    }

    // If there is an agent
    if (!isAgent) {
      a->setNode(target);
      return true;
    }
  }

  // failed
  a->setNode(a->getNode());
  return false;
}

Nodes PIBT::createCandidates(PIBT_Agent* a, Nodes CLOSE_NODE) {
  Nodes C;
  for (auto v : G->neighbor(a->getNode())) {
    if (!inArray(v, CLOSE_NODE)) C.push_back(v);
  }
  if (!inArray(a->getNode(), CLOSE_NODE)) C.push_back(a->getNode());
  return C;
}

Nodes PIBT::createCandidates(PIBT_Agent* a, Nodes CLOSE_NODE, Node* tmp) {
  CLOSE_NODE.push_back(tmp);
  Nodes C = createCandidates(a, CLOSE_NODE);
  CLOSE_NODE.erase(CLOSE_NODE.end() - 1);
  return C;
}

Node* PIBT::chooseNode(PIBT_Agent* a, Nodes C) {
  if (C.empty()) {
    std::cout << "error@PIBT::chooseNode, C is empty" << "\n";
    std::exit(1);
  }

  // randomize
  std::shuffle(C.begin(), C.end(), *MT);

  if (!a->hasGoal()) {
    if (inArray(a->getNode(), C)) {  // try to stay
      return a->getNode();
    } else {
      return C[0];  // random walk
    }
  }

  Nodes cs;
  int minCost = 10000;
  int cost;
  Node* g = a->getGoal();

  // fast implementation
  Nodes p = G->getPath(a->getNode(), g);
  if (p.size() > 1 && inArray(p[1], C)) return p[1];

  for (auto v : C) {
    cost = pathDist(v, g);
    if (cost < minCost) {
      minCost = cost;
      cs.clear();
      cs.push_back(v);
    } else if (cost == minCost) {
      cs.push_back(v);
    }
  }

  if (cs.size() == 1) return cs[0];

  // tie break
  bool contained;
  for (auto v : cs) {  // avoid tabu list
    contained = std::any_of(A.begin(), A.end(),
                            [v](PIBT_Agent* b){ return b->getNode() == v; });
    if (!contained) return v;
  }

  return cs[0];
}

void PIBT::updateC(Nodes& C, Node* target, Nodes CLOSE_NODE) {
  for (auto v : CLOSE_NODE) {
    auto itr2 = std::find_if(C.begin(), C.end(),
                             [v] (Node* u) { return v == u; });
    if (itr2 != C.end()) C.erase(itr2);
  }
}

std::string PIBT::logStr() {
  std::string str;
  str += "[solver] type:PIBT\n";
  str += Solver::logStr();
  return str;
}
