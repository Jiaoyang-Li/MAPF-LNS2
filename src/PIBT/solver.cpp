/*
 * solver.cpp
 *
 * Purpose: utility of solver class
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "solver.h"
#include <random>
#include "util.h"
#include <typeinfo>


Solver::Solver(Problem* _P) : P(_P) {
  std::random_device seed_gen;
  MT = new std::mt19937(seed_gen());
  init();
}
Solver::Solver(Problem* _P, std::mt19937* _MT) : P(_P), MT(_MT) {
  init();
}

Solver::~Solver() {}

void Solver::init() {
  G = P->getG();
  A = P->getA();
  int nodeNum = G->getNodesNum();
  dists = Eigen::MatrixXi::Zero(nodeNum, nodeNum);
}

void Solver::solveStart() {
  startT = std::chrono::system_clock::now();
}

void Solver::solveEnd() {
  endT = std::chrono::system_clock::now();
  elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>
    (endT-startT).count();

  // check consistency
  // 1. create path
  std::vector<Nodes> paths;
  for (auto a : A) {
    Nodes path;
    for (auto s : a->getHist()) path.push_back(s->v);
    paths.push_back(path);
  }
  // 2. check continuity, vertex/swap conflict
  for (int i = 0; i < A.size(); ++i) {
    for (int j = i + 1; j < A.size(); ++j) {
      if (paths[i].size() != paths[j].size()) {
        std::cout << "error@Solver, path size is different" << std::endl;
        std::exit(1);
      }
      for (int t = 0; t < paths[i].size(); ++t) {
        if (t > 0) {
          auto cands = paths[i][t-1]->getNeighbor();
          cands.push_back(paths[i][t-1]);
          if (!inArray(paths[i][t], cands)) {
            std::cout << "error@Solver, paths is not connected at t=" << t << ", "
                      << "agent " << i
                      << ", from " << paths[i][t-1]->getId()
                      << ", to " << paths[i][t]->getId()
                      << std::endl;
            std::exit(1);
          }
        }
        if (paths[i][t] == paths[j][t]) {
          std::cout << "error@Solver, vertex conflict at t=" << t << " between "
                    << i << " and " << j << std::endl;
          std::exit(1);
        }
        if (t > 0 && paths[i][t] == paths[j][t-1] && paths[i][t-1] == paths[j][t]) {
          std::cout << "error@Solver, swap conflict at t=" << t << " between "
                    << i << " and " << j << std::endl;
          std::exit(1);
        }
      }
    }
  }
}

void Solver::WarshallFloyd() {
  int nodeNum = G->getNodesNum();
  Nodes neighbor;
  int INF = 100000;
  dists = Eigen::MatrixXi::Ones(nodeNum, nodeNum) * INF;

  // initialize weight
  for (int i = 0; i < nodeNum; ++i) {
    neighbor = G->neighbor(G->getNodeFromIndex(i));
    for (auto v : neighbor) {
      dists(i, v->getIndex()) = 1;
    }
    dists(i, i) = 0;
  }

  // main loop
  for (int k = 0; k < nodeNum; ++k) {
    for (int i = 0; i < nodeNum; ++i) {
      for (int j = 0; j < nodeNum; ++j) {
        if (dists(i, j) > dists(i, k) + dists(k, j)) {
          dists(i, j) = dists(i, k) + dists(k, j);
        }
      }
    }
  }
}

int Solver::getMaxLengthPaths(Paths& paths) {
  if (paths.empty()) return 0;
  auto itr = std::max_element(paths.begin(), paths.end(),
                              [] (Nodes p1, Nodes p2) {
                                return p1.size() < p2.size(); });
  return itr->size();
}

void Solver::formalizePath(Paths& paths) {
  int maxLength = getMaxLengthPaths(paths);
  Node* g;
  for (int i = 0; i < paths.size(); ++i) {
    g = paths[i][paths[i].size() - 1];
    while (paths[i].size() != maxLength) paths[i].push_back(g);
  }
}

int Solver::pathDist(Node* s, Node* g) {
  // same place?
  if (s == g) return 0;

  // has already explored?
  int s_index = G->getNodeIndex(s);
  int g_index = G->getNodeIndex(g);
  int dist = dists(s_index, g_index);
  if (dist != 0) return dist;

  // new
  Nodes path = G->getPath(s, g);

  dist = path.size() - 1;  // without start node
  int index, d, cost;
  bool directed = G->isDirected();

  cost = dist;
  for (auto v : path) {
    index = G->getNodeIndex(v);
    d = dists(index, g_index);
    if ((index != g_index) && (d == 0)) {
      dists(index, g_index) = cost;
      // if not directed graph
      if (!directed) dists(g_index, index) = cost;
      --cost;
    } else if (d == cost) {
      break;
    } else {
      std::cout << "error@Solver::pathDist, "
                << s->getId() << " -> " << g->getId() << ", "
                << s->getPos() << " -> " << g->getPos() << ", "
                << "not optimal path is obtained" << "\n";
      std::exit(1);
    }
  }

  return dist;
}

int Solver::pathDist(Node* s, Node* g, Nodes &prohibited) {
  // same place?
  if (s == g) return 0;

  return G->getPath(s, g, prohibited).size() - 1;
}

std::string Solver::getKey(int t, Node* v) {
  std::string key = "";
  key += std::to_string(t);
  key += "-";
  key += std::to_string(v->getId());
  return key;
}

std::string Solver::getKey(AN* n) {
  return getKey(n->g, n->v);
}

std::string Solver::logStr() {
  std::string str;

  str += "[solver] solved:" + std::to_string(P->isSolved()) + "\n";
  str += "[solver] elapsed:" + std::to_string((int)elapsedTime) + "\n";
  str += "[solver] makespan:" + std::to_string(P->getTerminationTime()) + "\n";
  str += P->logStr();

  return str;
}
