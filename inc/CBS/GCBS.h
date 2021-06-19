#pragma once
#include "GCBSNode.h"
#include "SpaceTimeAStar.h"


class GCBS
{
public:
    bool randomRoot = false; // randomize the order of the agents in the root CT node

    /////////////////////////////////////////////////////////////////////////////////////
    // stats
    double runtime = 0;
    double runtime_generate_child = 0; // runtimr of generating child nodes
    double runtime_build_CT = 0; // runtimr of building constraint table
    double runtime_build_CAT = 0; // runtime of building conflict avoidance table
    double runtime_path_finding = 0; // runtime of finding paths for single agents
    double runtime_detect_conflicts = 0;

    uint64_t num_target_conflicts = 0;
    uint64_t num_standard_conflicts = 0;
    uint64_t num_adopt_bypass = 0; // number of times when adopting bypasses
    uint64_t num_HL_expanded = 0;
    uint64_t num_HL_generated = 0;

    GCBSNode* best_node = nullptr;
    vector<Path*> paths;

    /////////////////////////////////////////////////////////////////////////////////////////
    // set params
    void setTargetReasoning(bool t) { target_reasoning = t; }
    void setDisjointSplitting(bool d) { disjoint_splitting = d; }
    void setBypass(bool b) { bypass = b; } // 2-agent solver for heuristic calculation does not need bypass strategy.
    void setSavingStats(bool s) { save_stats = s; }

    ////////////////////////////////////////////////////////////////////////////////////////////
    // Runs the algorithm until the problem is solved or time is exhausted
    bool solve(double time_limit);
    void updatePaths(GCBSNode* curr);

    GCBS(vector<SingleAgentSolver*>& search_engines, int screen,
         const vector<PathTable>* path_tables);
    ~GCBS();

    // Save results
    void saveResults(const string &fileName, const string &instanceName) const;

    void clear(); // used for rapid random  restart

private:
    bool target_reasoning;  // using target reasoning
    bool disjoint_splitting;  // disjoint splitting
    bool bypass; // using Bypass1
    bool save_stats;

    list<GCBSNode*> allNodes_table;
    const vector<PathTable>* path_tables;

    pairing_heap< GCBSNode*, compare<GCBSNode::compare_node_by_d> > focal_list;

    int screen;
    double time_limit = -1;
    int collision_upperbound = MAX_COST;
    clock_t start;
    int num_of_agents;

    string getSolverName() const;

    // vector<Path> paths_found_initially;  // contain initial paths found
    // vector<MDD*> mdds_initially;  // contain initial paths found
    vector < SingleAgentSolver* > search_engines;  // used to find (single) agents' paths and mdd

    void addConstraints(const GCBSNode* curr, GCBSNode* child1, GCBSNode* child2) const;
    set<int> getInvalidAgents(const list<Constraint>& constraints); // return agents that violates the constraints
    //conflicts
    void findConflicts(GCBSNode& curr);
    void findConflicts(GCBSNode& curr, int a1, int a2);
    shared_ptr<Conflict> chooseConflict(const GCBSNode &node) const;
    static void copyConflicts(const list<shared_ptr<Conflict>>& conflicts,
                              list<shared_ptr<Conflict>>& copy, const list<int>& excluded_agent);

    inline void releaseNodes();

    // print and save
    void printResults() const;
    static void printConflicts(const GCBSNode &curr) ;

    bool validateSolution() const;
    inline int getAgentLocation(int agent_id, size_t timestep) const;

    vector<int> shuffleAgents() const;  //generate random permutation of agent indices



    // node operators
    inline void pushNode(GCBSNode* node);
    GCBSNode* selectNode();
    inline bool reinsertNode(GCBSNode* node);

    bool terminate();

    // high level search
    bool generateChild(GCBSNode* child, GCBSNode* curr);
    bool generateRoot();
    bool findPathForSingleAgent(GCBSNode* node, int ag);

    void printPaths() const;
};
