/*
 * pps.cpp
 *
 * Purpose: Parallel Push & Swap
 *
 * Sajid, Q., Luna, R., & Bekris, K. E. (2012).
 * Multi-Agent Pathfinding with Simultaneous Execution of Single-Agent Primitives.
 *
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "pps.h"
#include "util.h"

int PPS::s_uuid = 0;

PPS::PPS(Problem* _P) : Solver(_P) {
  init();
}
PPS::PPS(Problem* _P, std::mt19937* _MT) : Solver(_P, _MT) {
  init();
}

PPS::~PPS() {}

void PPS::init() {
  status = true;  // correct
  G->setRegFlg(true);
  for (auto a : A) {
    goals.push_back(a->getGoal());
    isTmpGoals.push_back(false);
  }

  // setup node lists, s.t., deg(v) >= 3
  for (auto v : G->getNodes()) {
    if (G->neighbor(v).size() >= 3) deg3nodes.push_back(v);
  }
}

bool PPS::solve() {
  solveStart();

  // l.1, initialize
  pushers = A;
  U.clear();
  L.clear();

  while (!P->isSolved()) {  // l.2
    update();
    if (!status) {
      solveEnd();
      return false;
    }
    P->update();
      if(time_limit&&((fsec)(std::chrono::system_clock::now()-startT)).count()>time_limit){
          break;
      }
  }

  solveEnd();
  return P->isSolved();
}

void PPS::update() {
  M.clear();  // l.3

  for (auto s : swapers) {  // l.4
    H.clear();
    if (CHECK_SWAPER(s) == CHECK::VALID) {  // check valid s
      if (SWAP(s) == RES::FAIL) {  // l.5
        status = false;
        return;
      }
    }
  }

  for (auto p : A) {  // l.6, push
    H.clear();
    if (CHECK_PUSHER(p) == CHECK::VALID) {
      PUSH(p, false);  // l.8
    }
  }

  // goal check
  for (auto a : A) {
    if (inArray(a, U)) {
      if (CHECK_GOAL(a) == CHECK::INVALID) {
        auto itrU = std::find(U.begin(), U.end(), a);
        U.erase(itrU);
      }
    } else {
      if (CHECK_GOAL(a) == CHECK::VALID) {
        U.push_back(a);  //l.9
      }
    }
  }

  // change pushers & swapers
  for (auto a : pusherToSwaper) {
    auto itr = std::find(pushers.begin(), pushers.end(), a);
    if (itr != pushers.end()) pushers.erase(itr);
  }
  pusherToSwaper.clear();

  for (auto a : swaperToPusher) {
    if (!inS(a)) pushers.push_back(a);  // check other's registration
  }
  swaperToPusher.clear();

  for (auto s : doneSwapers) {
    auto itr = std::find(swapers.begin(), swapers.end(), s);
    swapers.erase(itr);
    delete s;
  }
  doneSwapers.clear();
}

// individual agent push
RES PPS::PUSH(PIBT_Agent* c, bool swap) {
  Nodes T = {};
  return PUSH(c, T, swap);
}

RES PPS::PUSH(PIBT_Agent* c, Nodes &T, bool swap) {
  if (inArray(c, H)) return RES::FAIL;   // l.1
  if (inArray(c, M)) return RES::PAUSE;  // l.2
  if (swap == false && inArray(c, U)) return PAUSE;  // l.3

  auto itrU = std::find(U.begin(), U.end(), c);
  if (itrU != U.end()) U.erase(itrU);  // l.4

  Nodes pi;
  RES attempt;
  if (isTmpGoals[c->getId()]) {  // l.5, used when swapping, case 3
    S* s = getS(c);
    // error check
    if (!inArray(s, doneSwapers) && s->phase != SWAPPHASE::CLEARING) {
      std::cout << "error@PPS::PUSH, invalid swap phase, s : "
                << s->id << ", phase : " << s->phase << "\n";
      std::exit(1);
    }
    pi = SHORTEST_PATH(c->getNode(), c->getGoal(), T);  // l.6
    attempt = FEASIBLE(c, pi, T, swap);     // l.16

  } else {

    pi = SHORTEST_PATH(c, c->getGoal(), T); // l.8
    attempt = FEASIBLE(c, pi, T, swap);     // l.9

    // exclude swap detected agents
    if (attempt == RES::FAIL && !inArray(c, pusherToSwaper)) {  // l.12
      Nodes Y = CLOSEST_EMPTY_VERTICLES(c);

      if (!Y.empty()) {
        Node* g = c->getGoal();
        Node* e = *std::min_element(Y.begin(), Y.end(),
                                   [this, g] (Node* e1, Node* e2)
                                   { return this->pathDist(e1, g)
                                       < this->pathDist(e2, g); });  // l.14
        pi = SHORTEST_PATH(c, e, T);  // l.15
        attempt = FEASIBLE(c, pi, T, swap);  // l.16
      }
    }
  }

  if (attempt == RES::SUCCESS) {  // l.17
    move(c, pi);  // l.18
    M.push_back(c);  // l.18
  }

  return attempt;  // l.19
}

// swap agent push
RES PPS::PUSH(S* s) {
  PIBT_Agent* aH = s->agents[0];
  PIBT_Agent* aL = s->agents[1];
  Node* target = s->esv[0];

  if (inArray(aH, M)) return RES::PAUSE;
  if (inArray(aL, M)) return RES::PAUSE;

  // check which is the nearest
  if (s->agents.size() == 2) {
    if (pathDist(aH->getNode(), target) > pathDist(aL->getNode(), target)) {
      s->agents = { aL, aH };
      aH = s->agents[0];
      aL = s->agents[1];
    }
  }

  if (aH->getNode() == target) {
    return SWAP(s);
  }

  Nodes pi = SHORTEST_PATH(aH->getNode(), target);  // l.6

  RES attempt = FEASIBLE(s, pi);  // l.16

  if (attempt == RES::SUCCESS) {  // l.17
    move(s, pi);  // l.18
    for (auto a : s->agents) M.push_back(a);  // l.18
  }

  return attempt;  // l.19
}

RES PPS::FEASIBLE(PIBT_Agent* c, Nodes &pi, Nodes &T, bool swap) {
  if (pi.size() < 2) return RES::FAIL;  // l.1
  Node* v = pi[1];  // l.2

  if (inArray(c, M)) {
    return RES::FAIL;
  }

  if (reserved(v, M)) {  // l.3
    return RES::PAUSE;  // l.4
  }

  if (inArray(v, L)) {  // l.5, reserved
    return RES::PAUSE;  // l.6
  }

  auto itrA = std::find_if(A.begin(), A.end(),
                           [v] (PIBT_Agent* a) { return a->getNode() == v; });
  Nodes pi_one, pi_two;
  PIBT_Agent* a = nullptr;

  if (itrA != A.end() && swap == false) {  // l.10
    a = *itrA;
    if (inS(a)) return RES::FAIL;  // l.11

    pi_one = SHORTEST_PATH(c->getNode(), c->getGoal());  // l.12
    pi_two = SHORTEST_PATH(a->getNode(), a->getGoal());  // l.13

    if (DEPEND(pi_one, pi_two)) {  // l.15
      SETUP_SWAP(c, a);  // l.16 & 17
      return RES::FAIL;  // l.18
    }

  } else if (itrA != A.end() && swap == true) {  // l.19
    a = *itrA;
    if (inS(a)) {  // l.20
      PIBT_Agent* origin = c;
      if (!H.empty()) origin = H[0];
      if (inS(origin)) {
        if (CHECK_PRIORITY(origin, a) == CHECK::INVALID) {
          return RES::FAIL;  // l.22
        }
      }
    }
  } else if (itrA == A.end()) {
    return RES::SUCCESS;  // l.23
  }

  a = *itrA;
  H.push_back(c);
  return PUSH(a, T, swap);  // l.24, recursive push
}

RES PPS::FEASIBLE(S* s, Nodes &pi) {
  if (pi.size() < 2) return RES::FAIL;  // l.1
  Node* v = pi[1];  // l.2

  if (reserved(v, M)) return RES::PAUSE;  // l.3, 4
  if (inArray(v, L))  return RES::PAUSE;  // l.5, 6

  auto itrA = std::find_if(A.begin(), A.end(),
                           [v](PIBT_Agent* a) { return a->getNode() == v; });
  PIBT_Agent* a;
  if (itrA != A.end()) {  // l.19
    a = *itrA;
    if (inS(a)) {  // l.20
      if (CHECK_PRIORITY(s, a) == CHECK::INVALID) {  // l.21
        return RES::FAIL;  // l.22
      }
    }
  } else if (itrA == A.end()) {
    return RES::SUCCESS;  // l.23
  }

  a = *itrA;
  H.push_back(s->agents[0]);
  H.push_back(s->agents[1]);
  return PUSH(a, true);  // l.24, recursive push
}

CHECK PPS::CHECK_PUSHER(PIBT_Agent* a) {
  // exclude swaper
  if (!inArray(a, pushers)) {
    return CHECK::INVALID;
  }

  // agent who becomes swaper
  if (inArray(a, pusherToSwaper)) {
    return CHECK::INVALID;
  }

  return CHECK::VALID;
}

CHECK PPS::CHECK_SWAPER(S* s) {
  // error check
  int num = s->agents.size();
  if (num < 2 || 3 < num) {
    std::cout << "error@PPS::CHECK_SWAPER, swaper "
              << s->id << " has invalid agents" << "\n";
    std::exit(1);
  }

  // check temporaly resolved by other swapers
  if (inArray(s, doneSwapers)) {
    return CHECK::INVALID;
  }

  if ((int)s->phase >= (int)SWAPPHASE::EVAC_H) {
    return CHECK::VALID;
  }

  PIBT_Agent* aH = s->agents[0];  // center
  PIBT_Agent* aL = s->agents[1];
  Nodes neighbor = G->neighbor(aH->getNode());
  bool aL2aH = inArray(aL->getNode(), neighbor);

  bool aE2aH = true;
  if (num == 3) {
    PIBT_Agent* aE = s->agents[2];
    aE2aH = inArray(aE->getNode(), neighbor);
  }

  Nodes piH = SHORTEST_PATH(aH->getNode(), aH->getGoal());
  Nodes piL = SHORTEST_PATH(aL->getNode(), aL->getGoal());
  bool depend = DEPEND(piH, piL) || DEPEND(piL, piH);

  if (aL2aH && aE2aH && depend) return CHECK::VALID;

  // if separated
  ADD_DONE_SWAPERS(s);
  return CHECK::INVALID;
}

CHECK PPS::CHECK_GOAL(PIBT_Agent* a) {
  if (a->getNode() == goals[a->getId()]) return CHECK::VALID;
  return CHECK::INVALID;
}

void PPS::ADD_DONE_SWAPERS(S* s) {
  doneSwapers.push_back(s);
  PIBT_Agents agents = s->agents;
  if (agents.size() == 3) {
    PIBT_Agent* a3 = agents[2];
    isTmpGoals[a3->getId()] = false;
    P->assign(a3->getTask());
    a3->releaseTask();
    a3->setGoal(goals[a3->getId()]);
  }
  for (auto a : agents) {
    if (!inArray(a, pushers)) {
      pushers.push_back(a);
    } else {
      std::cout << "error@PPS::ADD_DONE_SWAPERS, "
                << "duplicated pusher : " << a->getId()
                << "\n";
      std::exit(1);
    }
  }
}

Nodes PPS::SHORTEST_PATH(Node* s, Node* g) {
  return G->getPath(s, g);
}

Nodes PPS::SHORTEST_PATH(Node* s, Node* g, Nodes prohibited) {
  return G->getPath(s, g, prohibited);
}

Nodes PPS::SHORTEST_PATH(PIBT_Agent* c, Node* g) {
  if (H.empty()) return G->getPath(c->getNode(), g);

  Nodes prohibited;
  for (auto a : H) prohibited.push_back(a->getNode());
  return G->getPath(c->getNode(), g, prohibited);
}

Nodes PPS::SHORTEST_PATH(PIBT_Agent* c, Node* g, Nodes& T) {
  if (H.empty()) return G->getPath(c->getNode(), g, T);

  Nodes prohibited = T;
  for (auto a : H) prohibited.push_back(a->getNode());
  return G->getPath(c->getNode(), g, prohibited);
}

RES PPS::SWAP(S* s) {
  // swapping phase
  if (s->phase == SWAPPHASE::EVAC_H ||
      s->phase == SWAPPHASE::EVAC_L ||
      s->phase == SWAPPHASE::SWAP_DONE) {
    SWAP_PRIMITIVES(s);
    return RES::SUCCESS;
  }

  PIBT_Agent* aH = s->agents[0];
  PIBT_Agent* aL = s->agents[1];
  Node* target = s->esv[0];

  if (s->agents.size() == 2) {  // l.1
    if (aH->getNode() != target) {  // l.2
      PUSH(s);  // l.3
    } else {  // swap starts
      if (CLEAR(s)) {  // l.7
        if (s->phase != SWAPPHASE::CLEARING) {
          SWAP_PRIMITIVES(s);  // l.6
        }
      } else {
        if (FIND_NEW_VERTEX(s) == RES::SUCCESS) {
          SWAP(s);
        } else {
          // for avoid collision
          M.push_back(aH);
          M.push_back(aL);
        }
      }
    }
  } else if (s->agents.size() == 3) {
    if (s->phase != SWAPPHASE::CLEARING) {
      std::cout << "error@PPS::SWAP, there are three agents" << "\n";
      std::exit(1);
    }
    PIBT_Agent* a = s->agents[2];  // l.10
    RES attempt = PUSH(a, s->area, true);  // l.11
    if (attempt == RES::FAIL) {  // l.12

      s->agents.erase(s->agents.end() - 1);  // l.13, pop
      isTmpGoals[a->getId()] = false;
      P->assign(a->getTask());
      a->releaseTask();
      a->setGoal(goals[a->getId()]);
      pushers.push_back(a);

      s->phase = SWAPPHASE::GO_TARGET;

      // avoid collision
      // M.push_back(s->agents[0]);
      // M.push_back(s->agents[1]);

      return FIND_NEW_VERTEX(s);  // l.14

    } else if (attempt == RES::SUCCESS) {

      if (a->getNode() == a->getGoal()) {

        s->agents.erase(s->agents.end() - 1);  // l.17
        P->assign(a->getTask());
        a->releaseTask();
        a->setGoal(goals[a->getId()]);  // l.17
        isTmpGoals[a->getId()] = false;

        swaperToPusher.push_back(a);

        if (s->agents.size() == 2) {
          s->phase = SWAPPHASE::GO_TARGET;
          SWAP(s);
        } else {
          std::cout << "error@PPS::SWAP, "
                    << "the number of agents should be two" << "\n";
          std::exit(1);
        }
      } else {

        PUSH(s);  // l.18

      }
    }
  } else {
    std::cout << "error@PPS::SWAP, "
              << "the number of agents should be"
              << " less than four" << "\n";
    std::exit(1);
  }

  return RES::SUCCESS;  // l.19
}

RES PPS::FIND_NEW_VERTEX(S* s) {
  // error check
  if (s->agents.size() != 2) {
    std::cout << "error@PPS::FIND_NEW_VERTEX, "
              << "the number of agents should be two" << "\n";
    std::exit(1);
  }

  if (s->esv.empty()) {
    return RES::FAIL;
  }

  // not highest swaper
  for (auto t : swapers) {
    if (s->id == t->id) continue;
    if (CHECK_PRIORITY(s, t) == CHECK::INVALID) {
      return RES::PAUSE;
    }
  }

  // highest swaper
  s->esv.erase(s->esv.begin());  // pop
  PIBT_Agents agents;
  PIBT_Agent* a0 = s->agents[0];
  PIBT_Agent* a1 = s->agents[1];
  Node* v = s->esv[0];
  if (pathDist(a0->getNode(), v) <= pathDist(a1->getNode(), v)) {
    agents = { a0, a1 };
  } else {
    agents = { a1, a0 };
  }
  s->agents = agents;

  return RES::SUCCESS;
}

void PPS::move(PIBT_Agent* a, Nodes &pi) {
  // error check
  if (pi.size() < 2) {
    std::cout << "error@PPS::move, pi does not enough nodes" << "\n";
    std::exit(1);
  }

  a->setNode(pi[1]);
}

void PPS::move(S* s, Nodes &pi) {
  // error check
  if (s->phase != SWAPPHASE::GO_TARGET &&
      s->phase != SWAPPHASE::CLEARING) {
    std::cout << "error@PPS::move, invalid movement" << "\n";
    std::exit(1);
  }

  s->agents[0]->setNode(pi[1]);
  s->agents[1]->setNode(pi[0]);
}

void PPS::SWAP_PRIMITIVES(S* s) {
  // error check
  if (s->phase != SWAPPHASE::EVAC_H &&
      s->phase != SWAPPHASE::EVAC_L &&
      s->phase != SWAPPHASE::SWAP_DONE) {
    std::cout << "error@PPS::SWAP_PRIMITIVES, "
              << "invalid phase was called" << "\n";
    std::exit(1);
  }
  if (s->agents.size() != 2) {
    std::cout << "error@PPS::SWAP_PRIMITIVES, "
              << "the size of agents must be 2, not "
              <<  s->agents.size() << "\n";
    std::exit(1);
  }

  PIBT_Agent* aH = s->agents[0];
  PIBT_Agent* aL = s->agents[1];

  if (inArray(aH, M) || inArray(aL, M)) {
    M.push_back(aH);
    M.push_back(aL);
    return;
  }

  switch (s->phase) {
  case SWAPPHASE::EVAC_H:
    aH->setNode(s->evacH);
    aL->setNode(s->target);
    s->phase = SWAPPHASE::EVAC_L;
    break;
  case SWAPPHASE::EVAC_L:
    aH->setNode(s->target);
    aL->setNode(s->evacL);
    s->phase = SWAPPHASE::SWAP_DONE;
    break;
  case SWAPPHASE::SWAP_DONE:
    aH->setNode(s->origin);
    aL->setNode(s->target);
    FINISH_SWAP(s);
    break;
  default:
    break;
  }
  M.push_back(aH);
  M.push_back(aL);
}

bool PPS::DEPEND(Nodes piA, Nodes piB) {
  // cond (a)
  if (inArray(piB[0], piA) && inArray(piB[piB.size()-1], piA)) return true;
  // cond (b)
  if (inArray(piB[0], piA) && inArray(piA[0], piB)) return true;

  return false;
}

Nodes PPS::getSortedEsv(PIBT_Agent* c) {
  Nodes lst = deg3nodes;
  Graph* _G = G;
  Node* v = c->getNode();
  // actually, should use pathDist, but for fast implementation
  std::sort(lst.begin(), lst.end(),
            [_G, v] (Node* v1, Node* v2)
            { return _G->dist(v, v1) < _G->dist(v, v2); });
  return lst;
}

void PPS::SETUP_SWAP(PIBT_Agent* c, PIBT_Agent* a) {
  // error check
  if (deg3nodes.size() == 0) {
    std::cout << "error@PPS::SETUP_SWAP, graph does not have esv" << "\n";
    std::exit(1);
  }

  Nodes sorted_esv = getSortedEsv(c);  // l.14
  Node* v = sorted_esv[0];
  PIBT_Agents agents;
  if (pathDist(c->getNode(), v) <= pathDist(a->getNode(), v)) {
    agents = {c, a};
  } else {
    agents = {a, c};
  }

  S* s = new S { s_uuid,
                 agents,  // [near (high), far (low)]
                 a->getNode(),  // [low] original pos
                 sorted_esv,
                 nullptr,  // target
                 nullptr,  // origin
                 nullptr,  // evacH
                 nullptr,  // evacL
                 {},       // area
                 SWAPPHASE::GO_TARGET };
  ++s_uuid;

  pusherToSwaper.push_back(c);
  pusherToSwaper.push_back(a);
  swapers.push_back(s);

  SWAP(s);
}

void PPS::FINISH_SWAP(S* s) {
  // error check
  if (s->agents.size() != 2) {
    std::cout << "error@PPS::FINISH_SWAP, the number of agetns is not two" << "\n";
    std::exit(1);
  }

  // clear L
  auto itrL = std::find(L.begin(), L.end(), s->target);
  L.erase(itrL);
  itrL = std::find(L.begin(), L.end(), s->origin);
  L.erase(itrL);
  itrL = std::find(L.begin(), L.end(), s->evacH);
  L.erase(itrL);
  itrL = std::find(L.begin(), L.end(), s->evacL);
  L.erase(itrL);

  swaperToPusher.push_back(s->agents[0]);
  swaperToPusher.push_back(s->agents[1]);
  doneSwapers.push_back(s);
}

bool PPS::CLEAR(S* s) {
  PIBT_Agent* aH = s->agents[0];
  PIBT_Agent* aL = s->agents[1];
  Node* target = s->esv[0];

  // error check
  if (s->phase != SWAPPHASE::GO_TARGET) {
    std::cout << "error@PPS::CLEAR, phase is invalid" << "\n";
    std::exit(1);
  }
  if (s->agents.size() != 2) {
    std::cout << "error@PPS::CLEAR, the number of agent is not two" << "\n";
    std::exit(1);
  }
  if (aH->getNode() != target) {
    std::cout << "error@PPS::CLEAR, "
              << "agent " << aH->getId() << " is not on node "
              << target->getId() << "\n";
    std::exit(1);
  }

  Nodes neighbor = G->neighbor(target);
  Node* evacH = nullptr;
  Node* evacL = nullptr;
  Node* aLPos = aL->getNode();

  // case 1. free nodes
  for (auto v : neighbor) {
    if (v == aLPos) continue;
    if (!isFree(v)) continue;

    if (!evacH) {
      evacH = v;
    } else if (!evacL) {
      evacL = v;
      s->phase = SWAPPHASE::EVAC_H;
      break;
    }
  }

  // case 2. try push
  if (!evacL) {
    for (auto v : neighbor) {
      if (inArray(v, L)) continue;
      if (v == aLPos) continue;
      if (evacH && v == evacH) continue;

      PIBT_Agent* a = *std::find_if(A.begin(), A.end(),
                               [v](PIBT_Agent* a) { return a->getNode() == v; });
      H.push_back(aH);
      H.push_back(aL);
      Nodes T = { evacH };
      RES attempt = PUSH(a, T, true);

      if (attempt != RES::SUCCESS) continue;

      if (!evacH) {
        evacH = v;
      } else {
        evacL = v;
        s->phase = SWAPPHASE::EVAC_H;
        break;
      }
    }
  }

  // case 3. evac
  if (evacH && !evacL) {
    for (auto v : neighbor) {
      if (inArray(v, L)) continue;
      if (v == aLPos) continue;

      // occuping agents
      auto itrU = std::find_if(A.begin(), A.end(),
                               [v](PIBT_Agent* a) { return a->getNode() == v; });
      if (itrU == A.end()) continue;
      PIBT_Agent* a3 = *itrU;

      if (inArray(a3, M)) continue;
      if (inS(a3)) {
        if (CHECK_PRIORITY(s, a3) == CHECK::INVALID) {
          continue;
        }
      }

      Node* a3target = nullptr;
      Nodes evacHneighbor = G->neighbor(evacH);
      for (auto u : evacHneighbor) {
        if (u == target) continue;
        if (u == a3->getNode()) continue;
        a3target = u;
        break;
      }
      if (!a3target) continue;

      Nodes pi = SHORTEST_PATH(evacH, a3target, { aLPos });
      if (pi.size() < 2) continue;  // there is no path

      // temporary change goal node
      aH->setGoal(aL->getNode());

      for (auto u : G->neighbor(aLPos)) {
        if (u == a3target) continue;
        if (u == target) continue;
        // temporary change goal node
        aL->setGoal(u);

        Nodes prohibited = { a3target, target, evacH };
        RES attempt = PUSH(aL, prohibited, true);

        if (attempt != RES::SUCCESS) continue;

        // resolve swap target
        if (inS(a3)) {
          S* t = getS(a3);
          doneSwapers.push_back(t);
          for (auto b : t->agents) {
            b->setGoal(goals[b->getId()]);
            isTmpGoals[b->getId()] = false;
            if (b != a3) pushers.push_back(b);
          }
        }

        auto itrU = std::find(U.begin(), U.end(), a3);
        if (itrU != U.end()) U.erase(itrU);

        PUSH(aH, true);  // to aLPos

        a3->setGoal(pi[1]);
        isTmpGoals[a3->getId()] = true;
        pusherToSwaper.push_back(a3);
        s->agents.push_back(a3);
        s->phase = SWAPPHASE::CLEARING;
        PUSH(a3, true);  // to target

        evacL = a3->getNode();

        break;
      }

      // resolve
      aL->setGoal(goals[aL->getId()]);
      aH->setGoal(goals[aH->getId()]);

      if (evacL) break;
    }
  }

  // CLEAR is success
  if (s->phase == SWAPPHASE::EVAC_H) {
    s->target = aH->getNode();
    s->origin = aL->getNode();
    s->evacH = evacH;
    s->evacL = evacL;

    L.push_back(s->target);
    L.push_back(s->origin);
    L.push_back(s->evacH);
    L.push_back(s->evacL);

    return true;

  } else if (s->phase == SWAPPHASE::CLEARING) {

    s->area = { aH->getNode(), aL->getNode(), evacL };
    return true;

  }

  return false;
}

bool PPS::reserved(Node* v, PIBT_Agents &M) {
  return std::any_of(M.begin(), M.end(),
                     [v](PIBT_Agent* a) { return a->getNode() == v; });
}

bool PPS::isFree(Node* v) {
  bool flg = std::none_of(A.begin(), A.end(),
                          [v](PIBT_Agent* a) { return a->getNode() == v; });
  flg &= !inArray(v, L);
  return flg;
}

bool PPS::inS(PIBT_Agent* a) {
  for (auto s : swapers) {
    if (inArray(s, doneSwapers)) continue;
    if (inArray(a, s->agents)) return true;
  }
  return false;
}

S* PPS::getS(PIBT_Agent* a) {
  auto itr = std::find_if(swapers.begin(), swapers.end(),
                          [a] (S* s) {
                            auto itrA = std::find(s->agents.begin(), s->agents.end(), a);
                            return itrA != s->agents.end(); });
  if (itr == swapers.end()) {
    std::cout << "error@PPS::getS, "
              << "corresponding swaper does not exist, "
              << "agent : " << a->getId() << "\n";
    std::exit(1);
  }
  return *itr;
}

CHECK PPS::CHECK_PRIORITY(PIBT_Agent* c, PIBT_Agent* a) {
  S* sC = getS(c);
  S* sA = getS(a);
  return CHECK_PRIORITY(sC, sA);
}

CHECK PPS::CHECK_PRIORITY(S* s, PIBT_Agent* a) {
  S* sA = getS(a);
  return CHECK_PRIORITY(s, sA);
}

CHECK PPS::CHECK_PRIORITY(S* sC, S* sA) {
  if ((int)sA->phase >= (int)SWAPPHASE::CLEARING) {
    return CHECK::INVALID;
  }
  if (sC->id <= sA->id) return CHECK::VALID;
  return CHECK::INVALID;
}

struct VD {
  Node* v;  // node
  int d;    // distance
};

Nodes PPS::CLOSEST_EMPTY_VERTICLES(PIBT_Agent* c) {
  Nodes empties;
  Nodes occupied;
  Nodes Y;
  Nodes C;

  // create occupied lists
  for (auto a : A) occupied.push_back(a->getNode());
  std::unordered_map<int, VD*> table;
  VD* n = new VD { c->getNode(), 0 };
  int id = n->v->getId();
  int d;

  std::unordered_set<int> OPEN = { id };
  table.emplace(id, n);

  while (!OPEN.empty()) {
    auto itr = std::min_element(OPEN.begin(), OPEN.end(),
                                [&table] (int a, int b)
                                { return table.at(a)->d < table.at(b)->d; });
    id = *itr;
    n = table.at(id);

    // check empty or not
    if (!inArray(n->v, occupied)) {
      if (Y.empty()) {
        d = n->d;
      } else if (d < n->d) {
        break;
      }
      // same distance
      Y.push_back(n->v);
    }

    // update list
    OPEN.erase(itr);

    C = G->neighbor(n->v);
    for (auto m : C) {
      id = m->getId();
      auto itr = table.find(id);
      if (itr == table.end()) {
        VD* l = new VD { m, n->d + 1 };
        table.emplace(id, l);
        OPEN.insert(id);
        continue;
      }
    }
  }

  return Y;
}

std::string PPS::logStr() {
  std::string str;
  str += "[solver] type:Parallel Push & Swap\n";
  str += Solver::logStr();
  return str;
}
