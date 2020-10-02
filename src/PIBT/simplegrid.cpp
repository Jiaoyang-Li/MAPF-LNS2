/*
 * simplegrid.cpp
 *
 * Purpose: simple graph, grid
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include <fstream>
#include <regex>
#include "util.h"
#include "simplegrid.h"

SimpleGrid::SimpleGrid(std::string _filename)
  : filename(_filename)
{
  init();
}

SimpleGrid::SimpleGrid(std::string _filename, std::mt19937* _MT)
  : Grid(_MT), filename(_filename)
{
  init();
}

SimpleGrid::~SimpleGrid() {}

void SimpleGrid::init() {
  setBasicParams();  // read w, h
  createNodes();     // create nodes, not edges
  createEdges();     // set neighbor
  setStartGoal();
}

void SimpleGrid::setBasicParams() {
  // read file
  std::ifstream file(filename);
  if (!file) {
    std::cout << "error@SimpleGrid::setBasicParams, file "
              << filename
              << " does not exist" << "\n";
    std::exit(1);
  }

  std::string line;
  std::smatch results;
  int w  = 0;
  int h = 0;

  std::regex r_height = std::regex(R"(height\s(\d+))");
  std::regex r_width = std::regex(R"(width\s(\d+))");
  std::regex r_map = std::regex(R"(map)");

  // read fundamental graph params
  while (getline(file, line)) {

    // for CRLF coding
    if (*(line.end()-1) == 0x0d) line.pop_back();

    if (std::regex_match(line, results, r_height)) {
      h = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_width)) {
      w = std::stoi(results[1].str());
    }
    if (std::regex_match(line, results, r_map)) {
      break;
    }
  }
  setSize(w, h);
  file.close();
}

void SimpleGrid::createNodes() {
  std::ifstream file(filename);
  if (!file) {
    std::cout << "error@SimpleGrid::createNodes, file "
              << filename
              << " does not exist" << "\n";
    std::exit(1);
  }

  std::string line;
  std::smatch results;
  std::regex r_map = std::regex(R"(map)");
  int w  = getW();
  int h = getH();
  int id;
  int j = 0;  // height
  bool mapStart = false;
  char s;

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end()-1) == 0x0d) line.pop_back();

    if (mapStart) {
      // width check
      if (line.size() != w) {
        std::cout << "error@SimpleGrid::createNodes, "
                  << "width is invalid, should be " << w <<  "\n";
        std::exit(1);
      }
      for (int i = 0; i < w; ++i) {
        s = line[i];
        id = j * w + i;
        if (s == 'T' or s == '@') continue;
        Node* v = new Node(id);
        v->setPos(j, i);
        nodes.push_back(v);
      }
      ++j;
    }

    if (std::regex_match(line, results, r_map)) mapStart = true;
  }
  file.close();

  // height check
  if (j != h) {
    std::cout << "error@SimpleGrid::createNodes, "
              << "height is invalid, shoudl be " << h <<  "\n";
    std::exit(1);
  }
}

void SimpleGrid::createEdges() {
  std::ifstream file(filename);
  if (!file) {
    std::cout << "error@SimpleGrid::createEdges, file "
              << filename
              << " does not exist" << "\n";
    std::exit(1);
  }

  std::string line;
  std::smatch results;
  std::regex r_map = std::regex(R"(map)");
  int w  = getW();
  int j = 0;  // height
  int id;
  bool mapStart = false;
  Nodes neighbor;
  std::string s;
  Node* v;

  std::regex r_direction = std::regex(R"([a-z])");
  std::regex r_obj   = std::regex(R"([@T])");
  std::regex r_up    = std::regex(R"([\.oaefgkmn])");
  std::regex r_down  = std::regex(R"([\.ocfhjklm])");
  std::regex r_left  = std::regex(R"([\.odgijlmn])");
  std::regex r_right = std::regex(R"([\.obehikln])");

  while (getline(file, line)) {
    // for CRLF coding
    if (*(line.end()-1) == 0x0d) line.pop_back();

    if (mapStart) {
      for (int i = 0; i < w; ++i) {
        id = j * w + i;
        s = line[i];

        // object
        if (std::regex_match(s, results, r_obj)) continue;

        // digraph, default is undirected graph
        if (!isDirected() && std::regex_match(s, results, r_direction)) {
          setDirected(true);
        }

        if (!existNode(id)) {
          std::cout << "error@SimpleGrid::createEdges, "
                    << "corresponding node does not exist, " << id << "\n";
          std::exit(1);
        }
        v = getNode(id);  // target node
        neighbor.clear();

        if (std::regex_match(s, results, r_up)) {
          if (existNode(id - w)) neighbor.push_back(getNode(id - w));
        }
        if (i != 0 && std::regex_match(s, results, r_left)) {
          if (existNode(id - 1)) neighbor.push_back(getNode(id - 1));
        }
        if (i != w - 1 && std::regex_match(s, results, r_right)) {
          if (existNode(id + 1)) neighbor.push_back(getNode(id + 1));
        }
        if (std::regex_match(s, results, r_down)) {
          if (existNode(id + w)) neighbor.push_back(getNode(id + w));
        }
        v->setNeighbor(neighbor);
      }
      ++j;
    }

    if (std::regex_match(line, results, r_map)) mapStart = true;
  }
  file.close();
}

void SimpleGrid::setStartGoal() {
  // all nodes are target
  starts = nodes;
  goals = nodes;
}

Node* SimpleGrid::getNewGoal(Node* v) {
  Node* u;
  do {
    u = randomChoose(goals, MT);
  } while (u == v);
  return u;
}

std::string SimpleGrid::logStr() {
  std::string str = Grid::logStr();
  str += "[graph] filename:" + filename + "\n";
  return str;
}
