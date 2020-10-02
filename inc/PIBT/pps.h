/*
 * pps.h
 *
 * Purpose: Parallel Push & Swap
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#pragma once
#include "solver.h"


enum RES { SUCCESS,
           FAIL,
           PAUSE };
enum CHECK { VALID, INVALID };
enum SWAPPHASE { GO_TARGET,
                 CLEARING,
                 EVAC_H,
                 EVAC_L,
                 SWAP_DONE };

struct S {
  int id;
  PIBT_Agents agents;  // priority : high priority, low priority
  Node* lowOriginalNode;  // [low] original pos, [high] goal pos
  Nodes esv;  // swap node candidates
  Node* target;  // swap node, deg(target) >= 3
  Node* origin;  // low pos when swap starts
  Node* evacH;  // [high] evacuation node
  Node* evacL;  // [low]  evacuation node
  Nodes area;   // swap area, using case 3
  SWAPPHASE phase;  // phase
};

class PPS : public Solver {
private:
  PIBT_Agents pushers;  // pushers
  std::vector<S*> swapers;   // swapers
  PIBT_Agents pusherToSwaper;  // tmp
  PIBT_Agents swaperToPusher;  // tmp
  std::vector<S*> doneSwapers;  // tmp

  PIBT_Agents M;  // moved agents
  PIBT_Agents U;  // goal agents
  Nodes L;   // reserved nodes
  PIBT_Agents H;  // previous pusher
  bool status;  // continue or not

  Nodes goals;  // final goal
  std::vector<bool> isTmpGoals;  // has temp goal
  Nodes deg3nodes;

  static int s_uuid;

  void init();

  RES SWAP(S* s);

  RES PUSH(PIBT_Agent* c, bool swap);
  RES PUSH(PIBT_Agent* c, Nodes &T, bool swap);
  RES PUSH(S* s);

  RES FEASIBLE(PIBT_Agent* c, Nodes &pi, Nodes& T, bool swap);
  RES FEASIBLE(S* s, Nodes &pi);

  Nodes SHORTEST_PATH(Node* s, Node* g);
  Nodes SHORTEST_PATH(PIBT_Agent* c, Node* g);
  Nodes SHORTEST_PATH(Node* s, Node* g, Nodes prohibited);
  Nodes SHORTEST_PATH(PIBT_Agent* c, Node* g, Nodes& T);

  bool DEPEND(Nodes piA, Nodes piB);

  void SETUP_SWAP(PIBT_Agent* c, PIBT_Agent* a);

  bool CLEAR(S* s);

  void SWAP_PRIMITIVES(S* s);
  void FINISH_SWAP(S* s);

  void move(PIBT_Agent* a, Nodes &pi);
  void move(S* s, Nodes &pi);

  bool reserved(Node* v, PIBT_Agents &M);
  bool isFree(Node* v);
  bool inS(PIBT_Agent* a);
  S* getS(PIBT_Agent* a);
  Nodes getSortedEsv(PIBT_Agent* c);

  CHECK CHECK_PRIORITY(S* s, PIBT_Agent* a);
  CHECK CHECK_PRIORITY(PIBT_Agent* c, PIBT_Agent* a);
  CHECK CHECK_PRIORITY(S* sC, S* sA);

  RES FIND_NEW_VERTEX(S* s);

  CHECK CHECK_PUSHER(PIBT_Agent* a);
  CHECK CHECK_SWAPER(S* s);
  CHECK CHECK_GOAL(PIBT_Agent* a);

  void ADD_DONE_SWAPERS(S* s);
  Nodes CLOSEST_EMPTY_VERTICLES(PIBT_Agent* c);

public:
  PPS(Problem* _P);
  PPS(Problem* _P, std::mt19937* _MT);
  ~PPS();

  bool solve();
  void update();

  std::string logStr();
};
