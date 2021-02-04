/*
 * graph.cpp
 *
 * Purpose: Graph
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "graph.h"
#include <random>
#include <unordered_set>
#include "util.h"
#include <boost/heap/fibonacci_heap.hpp>

Graph::Graph() {
  std::random_device seed_gen;
  MT = new std::mt19937(seed_gen());
  init();
}

Graph::Graph(std::mt19937* _MT) : MT(_MT) {
  init();
}

void Graph::init() {
  directed = false;
  regFlg = true;
}

Graph::~Graph() {
  for (auto v : nodes) delete v;
  nodes.clear();
  for (auto p : knownPaths) delete p.second;
  knownPaths.clear();
}

Node* Graph::getNode(int id) {
  auto itr = std::find_if(nodes.begin(), nodes.end(),
                          [id](Node* v){ return v->getId() == id; });
  // error check
  if (itr == nodes.end()) {
    std::cout << "error@Graph::getNode, "
              << "node index is over, " << id << "\n";
    std::exit(1);
  }

  return *itr;
}

Node* Graph::getNode(int x, int y) {
  auto itr = std::find_if(nodes.begin(), nodes.end(),
                          [x, y](Node* v)
                          { return v->getPos().x == x
                              && v->getPos().y == y; });
  if (itr == nodes.end()) return nullptr;
  return *itr;
}

bool Graph::existNode(int id) {
  auto itr = std::find_if(nodes.begin(), nodes.end(),
                          [id](Node* v){ return v->getId() == id; });
  return itr != nodes.end();
}

int Graph::getNodeIndex(Node* v) {
  return v->getIndex();
}

Nodes Graph::neighbor(Node* v) {
  return v->getNeighbor();
}

Nodes Graph::neighbor(int i) {
  return getNode(i)->getNeighbor();
}

Nodes Graph::getPath(Node* s, Node* g, Nodes &prohibitedNodes) {
  return {};
}

// regFlg : whether register
Nodes Graph::getPath(Node* _s, Node* _g,
                     Nodes &prohibitedNodes, int (*dist) (Node*, Node*))
{
  bool prohibited = !prohibitedNodes.empty();
  Nodes path, C;
  std::string key;

  // ==== fast implementation ====
  if (regFlg && !prohibited) {
    key = getKey(_s, _g);
    auto itrK = knownPaths.find(key);
    if (itrK != knownPaths.end()) {  // known
      path = itrK->second->path;
      return path;
    }
  }
  // =============================

  int f;
  bool invalid = true;

  // prepare node open hashtable
  boost::heap::fibonacci_heap<Fib_AN> OPEN;
  std::unordered_map<int, boost::heap::fibonacci_heap<Fib_AN>::handle_type> SEARCHED;
  std::unordered_set<int> CLOSE;
  AN* n = new AN { _s, 0, dist(_s, _g), nullptr };
  auto handle = OPEN.push(Fib_AN(n));
  SEARCHED.emplace(n->v->getId(), handle);

  while (!OPEN.empty()) {
    // argmin
    n = OPEN.top().node;

    // check goal condition
    if (n->v == _g) {
      invalid = false;
      break;
    }

    // ==== fast implementation ====
    key = getKey(n->v, _g);
    auto itrK = knownPaths.find(key);
    if (itrK != knownPaths.end()) {  // known
      Nodes kPath = itrK->second->path;
      bool valid = true;
      if (prohibited) {
        for (auto v : kPath) {
          if (inArray(v, prohibitedNodes)) {
            valid = false;
            break;
          }
        }
      }
      if (valid) {
        for (int i = 1; i < kPath.size(); ++i) {
          n = new AN { kPath[i], 0, 0, n };
        }
        invalid = false;
        break;
      }
    }
    // =============================

    // update list
    OPEN.pop();
    CLOSE.emplace(n->v->getId());

    // search neighbor
    C = neighbor(n->v);

    for (auto m : C) {
      if (prohibited && inArray(m, prohibitedNodes)) continue;
      if (CLOSE.find(m->getId()) != CLOSE.end()) continue;
      f = n->g + 1 + dist(m, _g);

      // ==== fast implementation ====
      if (regFlg) {
        key = getKey(m, _g);
        auto itrK = knownPaths.find(key);
        if (itrK != knownPaths.end()) {
          f = n->g + 1 + itrK->second->path.size() - 1;
        }
      }
      // =============================

      auto itrS = SEARCHED.find(m->getId());
      if (itrS == SEARCHED.end()) {  // new node
        AN* l = new AN { m, n->g + 1, f, n };
        auto handle = OPEN.push(Fib_AN(l));
        SEARCHED.emplace(l->v->getId(), handle);
      } else {
        auto handle = itrS->second;
        AN* l = (*handle).node;
        if (l->f > f) {
          l->g = n->g + 1;
          l->f = f;
          l->p = n;
          OPEN.increase(handle);
        }
      }
    }
  }

  if (invalid) return path;

  // back tracking
  while (n != nullptr) {
    path.push_back(n->v);
    n = n->p;
  }
  std::reverse(path.begin(), path.end());

  // register path
  if (regFlg && !prohibited) registerPath(path);

  return path;
}

std::string Graph::getKey(Node* s, Node* g) {
  int sIndex = getNodeIndex(s);
  int gIndex = getNodeIndex(g);
  std::string key = "";
  key += std::to_string(sIndex);
  key += "-";
  key += std::to_string(gIndex);
  return key;
}

void Graph::registerPath(const Nodes &path) {
  if (path.empty()) return;

  Nodes tmp = path;
  std::string key;

  Node *v1, *v2;
  do {
    v1 = tmp[0];
    v2 = tmp[tmp.size() - 1];
    key = getKey(v1, v2);
    KnownPath* knownPath = new KnownPath { v1, v2, tmp };
    knownPaths.emplace(key, knownPath);
    tmp.erase(tmp.begin());
  } while (tmp.size() > 2);
}

Paths Graph::getRandomStartGoal(int num) {
  if (num > starts.size() || num > goals.size()) {
    std::cout << "error@Graph::getStartGoal, over node size" << "\n";
    std::exit(1);
  }

  Paths points;
  Nodes ss(starts.size());
  Nodes gs(goals.size());
  bool flg;

  std::copy(starts.begin(), starts.end(), ss.begin());
  std::copy(goals.begin(),  goals.end(), gs.begin());

  while (true) {
    points.clear();
    std::shuffle(ss.begin(), ss.end(), *MT);
    std::shuffle(gs.begin(), gs.end(), *MT);

    flg = true;
    for (int i = 0; i < num; ++i) {
      if (ss[i] != gs[i]) {
        points.push_back({ ss[i], gs[i] });
      } else {
        flg = false;
        break;
      }
    }

    if (flg) break;
  }

  return points;
}
