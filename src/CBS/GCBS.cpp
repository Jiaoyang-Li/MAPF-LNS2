#include "GCBS.h"
#include <algorithm>    // std::shuffle
#include <random>      // std::default_random_engine
#include <chrono>       // std::chrono::system_clock
#include "SpaceTimeAStar.h"


// takes the paths_found_initially and UPDATE all (constrained) paths found for agents from curr to start
void GCBS::updatePaths(GCBSNode* curr)
{
    vector<bool> updated(num_of_agents, false);  // initialized for false
    while (curr != nullptr)
    {
        for (auto & path : curr->paths)
        {
            if (!updated[path.first])
            {
                paths[path.first] = &(path.second);
                updated[path.first] = true;
            }
        }
        curr = curr->parent;
    }
}

void GCBS::copyConflicts(const list<shared_ptr<Conflict >>& conflicts,
                        list<shared_ptr<Conflict>>& copy, const list<int>& excluded_agents)
{
    for (auto& conflict : conflicts)
    {
        bool found = false;
        for (auto a : excluded_agents)
        {
            if (conflict->a1 == a || conflict->a2 == a)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            assert(!conflict->constraint1.empty());
            assert(!conflict->constraint2.empty());
            copy.push_back(conflict);
        }
    }
}


void GCBS::findConflicts(GCBSNode& curr, int a1, int a2)
{
    int min_path_length = (int) (paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size());
    if (paths[a1]->size() != paths[a2]->size())
    {
        int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
        int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
        int loc1 = paths[a1_]->back().location;
        for (int timestep = min_path_length; timestep < (int)paths[a2_]->size(); timestep++)
        {
            int loc2 = paths[a2_]->at(timestep).location;
            if (loc1 == loc2)
            {
                shared_ptr<Conflict> conflict(new Conflict());
                if (target_reasoning)
                    conflict->targetConflict(a1_, a2_, loc1, timestep);
                else
                    conflict->vertexConflict(a1_, a2_, loc1, timestep);
                assert(!conflict->constraint1.empty());
                assert(!conflict->constraint2.empty());
                curr.conflicts.push_front(conflict);
                return;
            }
        }
    }
    for (int timestep = 0; timestep < min_path_length; timestep++)
    {
        int loc1 = paths[a1]->at(timestep).location;
        int loc2 = paths[a2]->at(timestep).location;
        if (loc1 == loc2)
        {
            shared_ptr<Conflict> conflict(new Conflict());
            if (target_reasoning && paths[a1]->size() == timestep + 1)
            {
                conflict->targetConflict(a1, a2, loc1, timestep);
            }
            else if (target_reasoning && paths[a2]->size() == timestep + 1)
            {
                conflict->targetConflict(a2, a1, loc1, timestep);
            }
            else
            {
                conflict->vertexConflict(a1, a2, loc1, timestep);
            }
            assert(!conflict->constraint1.empty());
            assert(!conflict->constraint2.empty());
            curr.conflicts.push_back(conflict);
            return;
        }
        else if (timestep < min_path_length - 1
                 && loc1 == paths[a2]->at(timestep + 1).location
                 && loc2 == paths[a1]->at(timestep + 1).location)
        {
            shared_ptr<Conflict> conflict(new Conflict());
            conflict->edgeConflict(a1, a2, loc1, loc2, timestep + 1);
            assert(!conflict->constraint1.empty());
            assert(!conflict->constraint2.empty());
            curr.conflicts.push_back(conflict); // edge conflict
            return;
        }
    }

}


void GCBS::findConflicts(GCBSNode& curr)
{
    clock_t t = clock();
    if (curr.parent != nullptr)
    {
        // Copy from parent
        auto new_agents = curr.getReplannedAgents();
        copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);
        copyConflicts(curr.parent->conflicts, curr.conflicts, new_agents);

        // detect new conflicts
        for (auto it = new_agents.begin(); it != new_agents.end(); ++it)
        {
            int a1 = *it;
            for (int a2 = 0; a2 < num_of_agents; a2++)
            {
                if (a1 == a2)
                    continue;
                bool skip = false;
                for (auto it2 = new_agents.begin(); it2 != it; ++it2)
                {
                    if (*it2 == a2)
                    {
                        skip = true;
                        break;
                    }
                }
                findConflicts(curr, a1, a2);
            }
        }
    }
    else
    {
        for (int a1 = 0; a1 < num_of_agents; a1++)
        {
            for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
            {
                findConflicts(curr, a1, a2);
            }
        }
    }
    curr.colliding_pairs = curr.conflicts.size();
    runtime_detect_conflicts += (double)(clock() - t) / CLOCKS_PER_SEC;
}


shared_ptr<Conflict> GCBS::chooseConflict(const GCBSNode &node) const
{
    if (screen == 3)
        printConflicts(node);
    shared_ptr<Conflict> choose;
    if (node.conflicts.empty())
        return nullptr;
    choose = node.conflicts.back();
    for (const auto& conflict : node.conflicts)
    {
        if (*choose < *conflict)
            choose = conflict;
    }
    return choose;
}

bool GCBS::findPathForSingleAgent(GCBSNode* node, int agent)
{
    // build constraint table
    auto t = clock();
    ConstraintTable constraint_table(search_engines[agent]->instance.num_of_cols,
                    search_engines[agent]->instance.map_size, &path_tables->at(agent));
    auto curr = node;
    while (curr->parent != nullptr)
    {
        constraint_table.insert2CT(curr->constraints, agent);
        curr = curr->parent;
    }
    runtime_build_CT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // build CAT
    t = clock();
    constraint_table.insert2CAT(agent, paths);
    runtime_build_CAT = (double)(clock() - t) / CLOCKS_PER_SEC;

    // find a path
    t = clock();
    Path new_path = search_engines[agent]->findPath(constraint_table);
    runtime_path_finding += (double)(clock() - t) / CLOCKS_PER_SEC;
    if (screen > 1)
        cout << "\t\t\tRuntime of single-agent search = " << (double)(clock() - t) / CLOCKS_PER_SEC <<
            "s with " << search_engines[agent]->getNumExpanded() << " expanded nodes" << endl;
    if (new_path.empty())
    {
        if (screen > 1)
            cout << "\t\t\t\tFail to find a path" << endl;
        return false;
    }
    assert(paths[agent] == nullptr or !isSamePath(*paths[agent], new_path));
    node->paths.emplace_back(agent, new_path);
    if (paths[agent] == nullptr)
        node->sum_of_costs += (int)new_path.size() - 1;
    else
        node->sum_of_costs += - (int)paths[agent]->size() + (int)new_path.size();
    paths[agent] = &node->paths.back().second;
    node->makespan = max(node->makespan, new_path.size() - 1);
    return true;
}

bool GCBS::generateChild(GCBSNode*  node, GCBSNode* parent)
{
    clock_t t1 = clock();
    node->parent = parent;
    node->sum_of_costs = parent->sum_of_costs;
    node->makespan = parent->makespan;
    node->depth = parent->depth + 1;

    auto agents = getInvalidAgents(node->constraints);
    assert(!agents.empty());
    for (auto agent : agents)
    {
        if (!findPathForSingleAgent(node, agent))
        {
            runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
            return false;
        }
    }

    findConflicts(*node);
    runtime_generate_child += (double)(clock() - t1) / CLOCKS_PER_SEC;
    return true;
}

inline void GCBS::pushNode(GCBSNode* node)
{
    num_HL_generated++;
    node->time_generated = num_HL_generated;
    allNodes_table.push_back(node);
    node->focal_handle = focal_list.push(node);
    if (best_node == nullptr or node->colliding_pairs < best_node->colliding_pairs)
        best_node = node;
}


inline bool GCBS::reinsertNode(GCBSNode* node)
{
    if (screen == 2)
    {
        cout << "	Reinsert " << "Node " << node->time_generated << " ( f = "<< node->sum_of_costs <<
        ", d = " << node->colliding_pairs << " ) with " << node->getNumNewPaths() << " new paths " << endl;
    }
    node->focal_handle = focal_list.push(node);
    if (node->colliding_pairs < best_node->colliding_pairs)
        best_node = node;
    return true;
}


GCBSNode* GCBS::selectNode()
{
    GCBSNode* curr = focal_list.top();
    focal_list.pop();
    return curr;
}


set<int> GCBS::getInvalidAgents(const list<Constraint>& constraints)  // return agents that violates the constraints
{
    set<int> agents;
    int agent, x, y, t;
    constraint_type type;
    assert(!constraints.empty());
    tie(agent, x, y, t, type) = constraints.front();

    if (type == constraint_type::LEQLENGTH)
    {
        assert(constraints.size() == 1);
        for (int ag = 0; ag < num_of_agents; ag++)
        {
            if (ag == agent)
                continue;
            for (int i = t; i < (int)paths[ag]->size(); i++)
            {
                if (paths[ag]->at(i).location == x)
                {
                    agents.insert(ag);
                    break;
                }
            }
        }
    }
    else if (type == constraint_type::POSITIVE_VERTEX)
    {
        assert(constraints.size() == 1);
        for (int ag = 0; ag < num_of_agents; ag++)
        {
            if (ag == agent)
                continue;
            if (getAgentLocation(ag, t) == x)
            {
                agents.insert(ag);
            }
        }
    }
    else if (type == constraint_type::POSITIVE_EDGE)
    {
        assert(constraints.size() == 1);
        for (int ag = 0; ag < num_of_agents; ag++)
        {
            if (ag == agent)
                continue;
            int curr = getAgentLocation(ag, t);
            int prev = getAgentLocation(ag, t - 1);
            if (prev == x || curr == y ||
                (prev == y && curr == x))
            {
                agents.insert(ag);
            }
        }

    }
    else
    {
        agents.insert(agent);
    }
    return agents;
}


void GCBS::printPaths() const
{
    for (int i = 0; i < num_of_agents; i++)
    {
        cout << "Agent " << i << " (" << paths[i]->size() - 1 << "): ";
        for (const auto & t : *paths[i])
            cout << t.location << "->";
        cout << endl;
    }
}



void GCBS::printResults() const
{
    cout << best_node->colliding_pairs << "," << best_node->sum_of_costs << "," << runtime << "," << endl;
}

void GCBS::saveResults(const string &fileName, const string &instanceName) const
{
    std::ifstream infile(fileName);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        ofstream addHeads(fileName);
        addHeads << "runtime,#colliding-pairs,#high-level expanded,#high-level generated," <<
                 "#low-level expanded,#low-level generated,#low-level reopened,#low-level runs" <<
                 "solution cost," <<
                 "#adopt bypasses," <<
                 "standard conflicts,target conflicts" <<
                 "runtime of detecting conflicts," <<
                 "runtime of building constraint tables,runtime of building CATs," <<
                 "runtime of path finding,runtime of generating child nodes," <<
                 "solver name,instance name" << endl;
        addHeads.close();
    }
    uint64_t num_LL_expanded = 0, num_LL_generated = 0, num_LL_reopened = 0, num_LL_runs = 0;
    for (auto & planner : search_engines)
    {
        planner->reset();
        num_LL_expanded += planner->accumulated_num_expanded;
        num_LL_generated += planner->accumulated_num_generated;
        num_LL_reopened += planner->accumulated_num_reopened;
        num_LL_runs += planner->num_runs;
    }
    ofstream stats(fileName, std::ios::app);
    stats << runtime << ",";
    if (!focal_list.empty())
        stats << focal_list.top()->colliding_pairs << "," <<
                  num_HL_expanded << "," << num_HL_generated << "," <<
                  num_LL_expanded << "," << num_LL_generated << "," << num_LL_reopened << "," << num_LL_runs << "," <<

                  focal_list.top()->sum_of_costs << "," <<

                  num_adopt_bypass << "," <<
                  num_standard_conflicts << "," << num_target_conflicts << "," <<

                  runtime_detect_conflicts << "," <<
                  runtime_build_CT << "," << runtime_build_CAT << "," <<
                  runtime_path_finding << "," << runtime_generate_child << "," <<

                  getSolverName() << "," << instanceName << endl;
    stats.close();
}

void GCBS::printConflicts(const GCBSNode &curr)
{
    for (const auto& conflict : curr.conflicts)
    {
        cout << *conflict << endl;
    }
    for (const auto& conflict : curr.conflicts)
    {
        cout << *conflict << endl;
    }
}


string GCBS::getSolverName() const
{
    string name;
    if (disjoint_splitting)
        name += "Disjoint ";
    name += "GCBS";
    if (target_reasoning)
        name += "+T";
    if (bypass)
        name += "+BP";
    name += " with " + search_engines[0]->getName();
    return name;
}


// return a solution once its total collisions has fewer collisions than _collision_upperbound
bool GCBS::solve(double _time_limit)
{
    // set timer
    start = clock();
    this->time_limit = _time_limit;

    if (screen > 0) // 1 or 2
    {
        string name = getSolverName();
        name.resize(35, ' ');
        cout << name << ": ";
        if (screen > 1)
            cout << endl;
    }


    assert(focal_list.empty());
    if(!generateRoot())
        return false;

    while (!terminate())
    {
        auto curr = selectNode();

        //Expand the node
        num_HL_expanded++;
        curr->time_expanded = num_HL_expanded;
        bool foundBypass = false;
        GCBSNode* child[2] = { new GCBSNode() , new GCBSNode() };
        curr->conflict = chooseConflict(*curr);
        addConstraints(curr, child[0], child[1]);

        if (screen > 1)
            cout << "	Expand " << "Node " << curr->time_generated << " ( f = "<< curr->sum_of_costs <<
            ", d = " << curr->colliding_pairs << " ) with " << curr->getNumNewPaths() << " new paths " << endl <<
                 "	on " << *(curr->conflict) << endl;

        bool solved[2] = { false, false };
        vector<vector<PathEntry>*> copy(paths);

        for (int i = 0; i < 2; i++)
        {
            if (i > 0)
                paths = copy;
            solved[i] = generateChild(child[i], curr);
            if (!solved[i])
            {
                delete (child[i]);
                continue;
            }
            else if (bypass && child[i]->colliding_pairs < curr->colliding_pairs) // Bypass1
            {
                if (i == 1 && !solved[0])
                    continue;
                foundBypass = true;
                num_adopt_bypass++;
                curr->conflicts = child[i]->conflicts;
                curr->colliding_pairs = child[i]->colliding_pairs;
                curr->conflict = nullptr;
                curr->sum_of_costs = child[i]->sum_of_costs;
                curr->makespan = child[i]->makespan;
                for (const auto& path : child[i]->paths) // update paths
                {
                    auto p = curr->paths.begin();
                    while (p != curr->paths.end())
                    {
                        if (path.first == p->first)
                        {
                            p->second = path.second;
                            paths[p->first] = &p->second;
                            break;
                        }
                        ++p;
                    }
                    if (p == curr->paths.end())
                    {
                        curr->paths.emplace_back(path);
                        paths[path.first] = &curr->paths.back().second;
                    }
                }
                if (screen > 1)
                {
                    cout << "	Update " << "Node " << curr->time_generated << " ( f = "<< curr->sum_of_costs <<
                    ", d = " << curr->colliding_pairs << " ) with " << curr->getNumNewPaths() << " new paths " << endl;
                }
                reinsertNode(curr);
                break;
            }
        }
        if (foundBypass)
        {
            for (auto & i : child)
            {
                delete i;
            }
        }
        else
        {
            for (int i = 0; i < 2; i++)
            {
                if (solved[i])
                {
                    pushNode(child[i]);
                    if (screen > 1)
                    {
                        cout << "\t\tGenerate " << "Node " << child[i]->time_generated <<
                        " ( f = "<< child[i]->sum_of_costs <<
                        ", d = " << child[i]->colliding_pairs << " ) with " <<
                        child[i]->getNumNewPaths() << " new paths " << endl;
                    }
                }
            }
            switch (curr->conflict->type)
            {
                case  conflict_type::TARGET:
                    num_target_conflicts++;
                    break;
                case conflict_type::STANDARD:
                    num_standard_conflicts++;
                    break;
                default:
                    break;
            }
            curr->clear();
        }
    }  // end of while loop
    updatePaths(best_node);
    if (screen > 0 )
        validateSolution();
    return best_node->colliding_pairs == 0;
}


void GCBS::addConstraints(const GCBSNode* curr, GCBSNode* child1, GCBSNode* child2) const
{
    if (disjoint_splitting && curr->conflict->type == conflict_type::STANDARD)
    {
        int first = (bool)(rand() % 2);
        if (first) // disjoint splitting on the first agent
        {
            child1->constraints = curr->conflict->constraint1;
            int a, x, y, t;
            constraint_type type;
            tie(a, x, y, t, type) = curr->conflict->constraint1.back();
            if (type == constraint_type::VERTEX)
            {
                child2->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
            }
            else
            {
                assert(type == constraint_type::EDGE);
                child2->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
            }
        }
        else // disjoint splitting on the second agent
        {
            child2->constraints = curr->conflict->constraint2;
            int a, x, y, t;
            constraint_type type;
            tie(a, x, y, t, type) = curr->conflict->constraint2.back();
            if (type == constraint_type::VERTEX)
            {
                child1->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_VERTEX);
            }
            else
            {
                assert(type == constraint_type::EDGE);
                child1->constraints.emplace_back(a, x, y, t, constraint_type::POSITIVE_EDGE);
            }
        }
    }
    else
    {
        child1->constraints = curr->conflict->constraint1;
        child2->constraints = curr->conflict->constraint2;
    }
}

GCBS::GCBS(vector<SingleAgentSolver*>& search_engines, int screen, const vector<PathTable>* path_tables = nullptr) :
           search_engines(search_engines), path_tables(path_tables),
           screen(screen), num_of_agents(search_engines.size()) {}


//generate random permuattion of agent indices
vector<int> GCBS::shuffleAgents() const
{
    vector<int> agents(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        agents[i] = i;
    }

    if (randomRoot)
    {
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(std::begin(agents), std::end(agents), g);
    }
    return agents;
}

bool GCBS::generateRoot()
{
    auto root = new GCBSNode();
    paths.resize(num_of_agents, nullptr);
    for (int i = 0; i < num_of_agents; i++)
    {
        auto succ = findPathForSingleAgent(root, i);
        assert(succ);
    }
    root->depth = 0;
    findConflicts(*root);

    pushNode(root);
    if (screen >= 2) // print start and goals
    {
        printPaths();
    }
    return true;
}

inline void GCBS::releaseNodes()
{
    focal_list.clear();
    for (auto& node : allNodes_table)
        delete node;
    allNodes_table.clear();
}

GCBS::~GCBS()
{
    releaseNodes();
}


bool GCBS::validateSolution() const
{
    // check whether the paths are feasible
    size_t soc = 0;
    int colliding_pairs = 0;
    for (int a1 = 0; a1 < num_of_agents; a1++)
    {
        soc += paths[a1]->size() - 1;
        for (int a2 = a1 + 1; a2 < num_of_agents; a2++)
        {
            size_t min_path_length = paths[a1]->size() < paths[a2]->size() ? paths[a1]->size() : paths[a2]->size();
            for (size_t timestep = 0; timestep < min_path_length; timestep++)
            {
                int loc1 = paths[a1]->at(timestep).location;
                int loc2 = paths[a2]->at(timestep).location;
                if (loc1 == loc2)
                {
                    if (best_node->colliding_pairs == 0)
                        cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
                    colliding_pairs++;
                    break;
                }
                else if (timestep < min_path_length - 1
                         && loc1 == paths[a2]->at(timestep + 1).location
                         && loc2 == paths[a1]->at(timestep + 1).location)
                {
                    if (best_node->colliding_pairs == 0)
                        cout << "Agents " << a1 << " and " << a2 << " collides at (" <<
                            loc1 << "-->" << loc2 << ") at timestep " << timestep << endl;
                    colliding_pairs++;
                    break;
                }
            }
            if (paths[a1]->size() != paths[a2]->size())
            {
                int a1_ = paths[a1]->size() < paths[a2]->size() ? a1 : a2;
                int a2_ = paths[a1]->size() < paths[a2]->size() ? a2 : a1;
                int loc1 = paths[a1_]->back().location;
                for (size_t timestep = min_path_length; timestep < paths[a2_]->size(); timestep++)
                {
                    int loc2 = paths[a2_]->at(timestep).location;
                    if (loc1 == loc2)
                    {
                        if (best_node->colliding_pairs == 0)
                            cout << "Agents " << a1 << " and " << a2 << " collides at " << loc1 << " at timestep " << timestep << endl;
                        colliding_pairs++;
                        break;
                    }
                }
            }
        }
    }
    if ((int)soc != best_node->sum_of_costs)
    {
        cerr << "The solution cost is wrong!" << endl;
        return false;
    }
    if (colliding_pairs != best_node->colliding_pairs)
    {
        cerr << "The number of collisions is wrong!" << endl;
        return false;
    }
    return true;
}

inline int GCBS::getAgentLocation(int agent_id, size_t timestep) const
{
    size_t t = max(min(timestep, paths[agent_id]->size() - 1), (size_t)0);
    return paths[agent_id]->at(t).location;
}


// used for rapid random  restart
void GCBS::clear()
{
    releaseNodes();
    paths.clear();
}

bool GCBS::terminate()
{
    runtime = (double)(clock() - start) / CLOCKS_PER_SEC;
    if (focal_list.empty() || focal_list.top()->colliding_pairs == 0 || runtime > time_limit)
    {
        if (screen > 0) // 1 or 2
            printResults();
        return true;
    }
    return false;
}