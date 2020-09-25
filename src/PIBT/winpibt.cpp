/*
 * winpibt.cpp
 *
 * Purpose: Enhanced PIBT
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "winpibt.h"
#include <algorithm>
#include "util.h"


winPIBT::winPIBT(Problem* _P, int _w, bool _softmode)
  : Solver(_P), w(_w), softmode(_softmode)
{
  init();
}

winPIBT::winPIBT(Problem* _P, int _w, bool _softmode, std::mt19937* _MT)
  : Solver(_P, _MT), w(_w), softmode(_softmode)
{
  init();
}

winPIBT::~winPIBT() {}

void winPIBT::init() {
  G->setRegFlg(true);

  // initialize priroirty and paths
  int agentNum = A.size();
  for (int i = 0; i < agentNum; ++i) {
    epsilon.push_back((float)i / agentNum);
    eta.push_back(0);
    priority.push_back(epsilon[i] + eta[i]);
    PATHS.push_back({A[i]->getNode()});
    L.push_back(0);
  }
}

bool winPIBT::solve() {
  solveStart();

  int t = 0;
  int t_sup = 0;
  int i, _w;

  while (!P->isSolved()) {
    allocate();
    updatePriority();
    std::vector<int> U(A.size());
    std::iota(U.begin(), U.end(), 0);
    std::sort(U.begin(), U.end(),
              [this] (int i, int j)
              {return this->priority[i] > this->priority[j];} );

    for (int j = 0; j < U.size(); ++j) {
      i = U[j];
      if (ell(i) <= t) {

        if (A[i]->hasTask()) {
          _w = w;
        } else {
          _w = 1;
        }

        if (j == 0) {
          winpibt(A[i], t + _w, softmode);
        } else {
          winpibt(A[i], std::min(t + _w, t_sup), softmode);
        }
      }

      if (j == 0) {
        t_sup = ell(i);
      } else {
        t_sup = std::min(t_sup, ell(i));
      }
    }

    // update path
    for (int i = 0; i < A.size(); ++i) A[i]->setNode(PATHS[i][t+1]);

    P->update();
    if (P->getTimestep() >= P->getTimestepLimit()) break;

    ++t;
  }

  solveEnd();
  return true;
}

void winPIBT::allocate() {
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

int winPIBT::ell(PIBT_Agent* a) {
  return ell(a->getId());
}

int winPIBT::ell(int id) {
  return L[id];
}

void winPIBT::updateGoal(PIBT_Agent* a) {
  // Node* g;
  // if (a->hasGoal() && lastNode(a) == a->getGoal()) {
  //   if (a->hasTask()) {
  //     g = a->getTask()->getNext(lastNode(a));
  //     if (g) {
  //       a->setGoal(g);
  //     }
  //   }
  // }
}

bool winPIBT::winpibt(PIBT_Agent* a, int t_tmp, bool varphi)
{
  int l = ell(a);
  if (l >= t_tmp) return true;

  Node* v;
  int i = a->getId();

  updateGoal(a);
  Node* g = getGoal(a);

  if (varphi && lastNode(i) == g) {
    PATHS[i].push_back(g);
    L[i] += 1;
    return true;
  }

  int t_max = getTmax(t_tmp);

  Nodes path = getPath(a, g, l, t_max);

  if (path.empty()) {
    v = PATHS[i][l];
    for (int _t = l + 1; _t <= t_tmp; ++_t) {
      PATHS[i].push_back(v);
    }
    L[i] = t_tmp;
    return false;
  }

  int _t = l + 1;

  // future information
  int t_dtmp = t_tmp;
  for (int j = _t; j <= t_tmp; ++j) {
    PATHS[i].push_back(path[j - l]);
    if (varphi && path[j - l] == g) {
      t_dtmp = j;
      break;
    }
  }

  while (_t <= t_dtmp) {
    v = PATHS[i][_t];
    L[i] = _t;

    auto itrA = findPITargetAgent(v, _t - 1);
    while (itrA != A.end()) {
      winpibt(*itrA, ell(*itrA) + 1, false);
      itrA = findPITargetAgent(v, _t - 1);
    }
    itrA = findPITargetAgent(v, _t);

    if (itrA != A.end()) {
      if (!winpibt(*itrA, _t, false)) {
        for (int j = _t; j <= t_dtmp; ++j) {
          PATHS[i].erase(PATHS[i].end() - 1);
        }
        L[i] = _t - 1;
        Nodes newPath = getPath(a, g, _t - 1, t_max);

        if (newPath.empty()) {

          v = PATHS[i][_t - 1];
          for (int __t = _t; __t <= t_dtmp; ++__t) {
            PATHS[i].push_back(v);
          }
          L[i] = t_dtmp;

          return false;

        } else {

          t_dtmp = t_tmp;
          for (int j = _t; j <= t_dtmp; ++j) {
            PATHS[i].push_back(newPath[j - _t + 1]);
            if (varphi && newPath[j - _t + 1] == g) {
              t_dtmp = j;
              break;
            }
          }

          continue;
        }
      }
    }

    if (v == g) {
      if (varphi) return true;

      updateGoal(a);
      g = getGoal(a);
      if (_t < t_dtmp) {
        Nodes newPath = getPath(a, g, _t, t_max);
        for (int j = _t + 1; j <= t_dtmp; ++j) {
          PATHS[i][j] = newPath[j - _t];
        }
      }
    }

    ++_t;
  }

  return true;
}

PIBT_Agents::iterator winPIBT::findPITargetAgent(Node* v, int t) {
  auto itr = std::find_if(A.begin(), A.end(),
                          [this, v, t] (PIBT_Agent* b) {
                            int id = b->getId();
                            int j = this->ell(id);
                            if (j >= t) return false;
                            if (this->lastNode(id) != v) return false;
                            return true;
                          });
  return itr;
}

Node* winPIBT::lastNode(PIBT_Agent* a) {
  return lastNode(a->getId());
}

Node* winPIBT::lastNode(int id) {
  return PATHS[id][ell(id)];
}

Node* winPIBT::getGoal(PIBT_Agent* a) {
  if (a->hasGoal()) return a->getGoal();
  return lastNode(a);
}

Nodes winPIBT::getPath(PIBT_Agent* a, Node* g, int t1, int t2) {
  if (t2 <= t1 || t2 <= ell(a)) {
    std::cout << "error@winPIBT::getPath"
              << ", ell : " << ell(a)
              << ", t1 : " << t1
              << ", t2 : " << t2
              << "\n";
    std::exit(1);
  }

  int t = t1;
  int id = a->getId();
  Node* s = PATHS[id][t1];  // start pos
  Nodes path, tmpPath;
  Nodes unprefarrable;

  int f = 0;
  std::string key;
  AN *l, *n;
  std::unordered_set<std::string> OPEN;
  std::unordered_map<std::string, AN*> table;
  Nodes C;

  l = new AN { s, true, t, pathDist(s, g), nullptr };
  key = getKey(t, s);
  table.emplace(key, l);
  OPEN = { key };

  while (!OPEN.empty()) {
    // argmin
    auto itr1 = std::min_element(OPEN.begin(), OPEN.end(),
                                 [&table, this, id]
                                 (std::string a, std::string b)
                                 { auto eleA = table.at(a);
                                   auto eleB = table.at(b);
                                   bool flgA, flgB;
                                   if (eleA->f == eleB->f) {
                                     for (int i = 0; i < this->PATHS.size(); ++i) {
                                       if (i == id) continue;
                                       flgA = false;
                                       flgB = false;
                                       if (this->ell(i) < eleA->t &&
                                           this->lastNode(i) == eleA->v) {
                                         flgA = true;
                                       }
                                       if (this->ell(i) < eleB->t &&
                                           this->lastNode(i) == eleB->v) {
                                         flgB = true;
                                       }
                                       if (flgA && !flgB) return false;
                                       if (!flgA && flgB) return true;
                                     }
                                     flgA = false;
                                     flgB = false;
                                     if (eleA->p && eleA->v == eleA->p->v) {
                                       flgA = true;
                                     }
                                     if (eleB->p && eleB->v == eleB->p->v) {
                                       flgB = true;
                                     }
                                     if (flgA && !flgB) return true;
                                     if (!flgA && flgB) return false;
                                     return eleA->t < eleB->t;
                                   }
                                   return eleA->f < eleB->f; });

    key = *itr1;
    n = table.at(key);

    // check goal
    if (n->t >= t2) break;

    // fast implementation
    tmpPath = G->getPath(n->v, g);
    while (n->t + tmpPath.size() - 1 < t2) tmpPath.push_back(g);
    while (n->t + tmpPath.size() - 1 > t2) tmpPath.erase(tmpPath.end() - 1);

    if (checkValidPath(id, tmpPath, n->t, t2)) {
      // for path efficiency
      bool interfere = false;
      int k;
      for (int i = 0; i < PATHS.size(); ++i) {
        if (i == id) continue;
        k = PATHS[i].size() - 1;
        for (int j = 1;  j < tmpPath.size(); ++j) {
          if (k >= n->t + j) continue;
          if (lastNode(i) == tmpPath[j]) {
            interfere = true;
            break;
          }
        }
        if (interfere) break;
      }

      if (!interfere) {
        tmpPath.erase(tmpPath.begin());
        for (auto v : tmpPath) n = new AN { v, true, n->t + 1, 0, n };
        break;
      }
    }

    // update list
    n->open = false;
    OPEN.erase(itr1);

    // search neighbor
    C = G->neighbor(n->v);
    C.push_back(n->v);

    for (auto m : C) {
      // check other paths
      t = n->t + 1;

      // collision check
      tmpPath = { n->v, m };
      if (!checkValidPath(id, tmpPath, n->t, t2)) continue;

      key = getKey(t, m);
      auto itr2 = table.find(key);
      if (itr2 == table.end()) {
        table.emplace(key, new AN { m, true, t, 100000, n });
      }
      l = table.at(key);
      if (!l->open) continue;

      f = t + pathDist(m, g);
      if (l->f > f) {
        l->t = t;
        l->f = f;
        l->p = n;
      }

      OPEN.insert(key);
    }
  }

  // back tracking
  if (n->t >= t2) {  // check failed or not
    while (n->p) {
      path.push_back(n->v);
      n = n->p;
    }
    path.push_back(s);
    std::reverse(path.begin(), path.end());
  }

  return path;
}

bool winPIBT::checkValidPath(int id, Nodes &path, int t1, int t2) {
  int k;
  Node *v1, *v2;

  for (int j = 1; j < path.size(); ++j) {
    v1 = path[j-1];
    v2 = path[j];
    for (int i = 0; i < A.size(); ++i) {
      if (id == i) continue;

      k = ell(i);
      for (int _t = j + t1 + 1; _t <= t2; ++_t) {
        if (k < _t) break;
        if (PATHS[i][_t] == v2) {  // collision
          return false;
        }
      }

      if (PATHS[i].size() - 1 < j + t1) continue;
      if (PATHS[i][j + t1] == v2) {  // collision
        return false;
      }
      if (v1 == PATHS[i][j + t1] && v2 == PATHS[i][j + t1 - 1]) {
        return false;
      }

    }
  }

  return true;
}

int winPIBT::getTmax(int t_tmp) {
  auto itrP = std::max_element(PATHS.begin(), PATHS.end(),
                               [] (Nodes a, Nodes b)
                               { return a.size() < b.size(); });
  int size = itrP->size() - 1;
  if (t_tmp > size) return t_tmp;
  return size;
}

void winPIBT::updatePriority() {
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
  }
}

std::string winPIBT::logStr() {
  std::string str;
  str += "[solver] type:winPIBT-" + std::to_string(w) + "\n";
  str += "[solver] softmode:" + std::to_string(softmode) + "\n";
  str += Solver::logStr();
  return str;
}
