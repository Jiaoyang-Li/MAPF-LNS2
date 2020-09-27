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
#include "assert.h"

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
      assert(false);
    std::cout << "error@Graph::getNode, "
              << "node index is over, " << id << "\n";
    std::exit(1);
  }

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

struct AN {  // Astar Node
  Node* v;
  bool open;
  int cost;
  int f;
  AN* p;
};

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

  // known path or not
  if (regFlg && !prohibited) {
    key = getKeyForKnownPath(_s, _g);
    auto itrK = knownPaths.find(key);
    if (itrK != knownPaths.end()) {  // known
      path = itrK->second->path;
      return path;
    }
  }

  // prepare node open hashtable
  std::unordered_map<int, AN*> table;
  AN* t = new AN { _s, true, 0, dist(_s, _g), nullptr };
  table.emplace(getNodeIndex(_s), t);

  struct AN* n = nullptr;
  struct AN* l = nullptr;
  int f, index;

  std::unordered_set<int> OPEN = { getNodeIndex(_s) };  // ID list

  while (!OPEN.empty()) {
    // argmin
    auto itr = std::min_element(OPEN.begin(), OPEN.end(),
                                [&table] (int a, int b)
                                { auto eleA = table.at(a);
                                  auto eleB = table.at(b);
                                  if (eleA->f == eleB->f) {
                                    return eleA->cost > eleB->cost;
                                  }
                                  return eleA->f < eleB->f; });

    index = *itr;
    n = table.at(index);

    // check goal condition
    if (n->v == _g) break;

    // already known?
    key = getKeyForKnownPath(n->v, _g);
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
          n = new AN { kPath[i], true, 0, 0, n };
        }
        break;
      }
    }

    // update list
    n->open = false;
    OPEN.erase(itr);

    // search neighbor
    C = neighbor(n->v);

    for (auto m : C) {
      if (prohibited && inArray(m, prohibitedNodes)) continue;
      index = getNodeIndex(m);

      auto itr = table.find(index);
      if (itr == table.end()) {
        AN* t = new AN { m, true, 0, 100000, nullptr };
        table.emplace(index, t);
      }
      l = table.at(index);
      if (!l->open) continue;

      f = n->cost + dist(m, _g);
      if (regFlg) {
        key = getKeyForKnownPath(m, _g);
        auto itrK = knownPaths.find(key);
        if (itrK != knownPaths.end()) {
          f = n->cost + itrK->second->path.size() - 1;
        } else if (prohibited) {
          key = getKeyForKnownPath(m, _g);
          if (itrK != knownPaths.end()) {
            f = n->cost + itrK->second->path.size() - 1;
          }
        }
      }

      if (l->f > f) {
        l->cost = n->cost + 1;
        l->f = f;
        l->p = n;
      }
      OPEN.insert(index);
    }
  }

  // back tracking
  while (n->p) {
    path.push_back(n->v);
    n = n->p;
  }
  path.push_back(_s);
  std::reverse(path.begin(), path.end());

  // register path
  if (regFlg && !prohibited) registerPath(path);

  return path;
}

std::string Graph::getKeyForKnownPath(Node* s, Node* g) {
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
    key = getKeyForKnownPath(v1, v2);
    KnownPath* knownPath = new KnownPath { v1, v2, tmp };
    knownPaths.emplace(key, knownPath);
    tmp.erase(tmp.begin());
  } while (tmp.size() > 2);
}

Paths Graph::getStartGoal(int num) {
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
