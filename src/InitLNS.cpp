#include "InitLNS.h"
#include <queue>

InitLNS::InitLNS(const Instance& instance, vector<Agent>& agents, double time_limit, string init_algo_name,
         string replan_algo_name, int neighbor_size, int screen) :
         instance(instance), agents(agents), time_limit(time_limit), init_algo_name(init_algo_name),
         replan_algo_name(replan_algo_name), neighbor_size(neighbor_size),
         screen(screen), path_table(instance.map_size), collision_graph(agents.size()),
         replan_time_limit(time_limit)
         {}

bool InitLNS::run()
{
    // only for statistic analysis, and thus is not included in runtime
    sum_of_distances = 0;
    for (const auto & agent : agents)
    {
        sum_of_distances += agent.path_planner.my_heuristic[agent.path_planner.start_location];
    }

    initial_solution_runtime = 0;
    start_time = Time::now();
    bool succ = getInitialSolution();
    initial_solution_runtime = ((fsec)(Time::now() - start_time)).count();
    iteration_stats.emplace_back(neighbor.agents.size(),
            initial_sum_of_costs, initial_solution_runtime, init_algo_name, 0, num_of_colliding_pairs);
    runtime = initial_solution_runtime;

    while (runtime < time_limit and num_of_colliding_pairs > 0)
    {
        runtime =((fsec)(Time::now() - start_time)).count();

        generateNeighborByCollisionGraph();

        // store the neighbor information
        neighbor.old_paths.resize(neighbor.agents.size());
        neighbor.old_sum_of_costs = 0;
        neighbor.old_colliding_pairs.clear();
        for (int i = 0; i < (int)neighbor.agents.size(); i++)
        {
            int a = neighbor.agents[i];
            if (replan_algo_name == "PP")
                neighbor.old_paths[i] = agents[a].path;
            path_table.deletePath(neighbor.agents[i], agents[a].path);
            neighbor.old_sum_of_costs += (int) agents[a].path.size() - 1;
            for (auto j: collision_graph[a])
            {
                neighbor.old_colliding_pairs.emplace(min(a, j), max(a, j));
            }
        }
        if (screen >= 2)
            cout << "Old colliding pairs = " << neighbor.old_colliding_pairs.size() << endl;

        if (replan_algo_name == "PP")
            succ = runPP();
        else
        {
            cerr << "Wrong replanning strategy" << endl;
            exit(-1);
        }

        if (ALNS) // update destroy heuristics
        {
            if (neighbor.colliding_pairs.size() < neighbor.old_colliding_pairs.size())
                destroy_weights[selected_neighbor] =
                        reaction_factor * (neighbor.old_colliding_pairs.size() - neighbor.colliding_pairs.size()) / neighbor.agents.size()
                        + (1 - reaction_factor) * destroy_weights[selected_neighbor];
            else
                destroy_weights[selected_neighbor] =
                        (1 - decay_factor) * destroy_weights[selected_neighbor];
        }

        if (succ) // update collision graph
        {
            num_of_colliding_pairs += (int)neighbor.colliding_pairs.size() - (int)neighbor.old_colliding_pairs.size();
            for(const auto& agent_pair : neighbor.old_colliding_pairs)
            {
                collision_graph[agent_pair.first].erase(agent_pair.second);
                collision_graph[agent_pair.second].erase(agent_pair.first);
            }
            for(const auto& agent_pair : neighbor.colliding_pairs)
            {
                collision_graph[agent_pair.first].emplace(agent_pair.second);
                collision_graph[agent_pair.second].emplace(agent_pair.first);
            }
            if (screen >= 2)
                printCollisionGraph();
        }
        runtime = ((fsec)(Time::now() - start_time)).count();
        sum_of_costs += neighbor.sum_of_costs - neighbor.old_sum_of_costs;
        if (screen >= 1)
            cout << "Iteration " << iteration_stats.size() << ", "
                 << "group size = " << neighbor.agents.size() << ", "
                 << "colliding pairs = " << num_of_colliding_pairs << ", "
                 << "solution cost = " << sum_of_costs << ", "
                 << "remaining time = " << time_limit - runtime << endl;
        iteration_stats.emplace_back(neighbor.agents.size(), sum_of_costs, runtime, replan_algo_name,
                                     0, num_of_colliding_pairs);
    }


    average_group_size = - iteration_stats.front().num_of_agents;
    for (const auto& data : iteration_stats)
        average_group_size += data.num_of_agents;
    if (average_group_size > 0)
        average_group_size /= (double)(iteration_stats.size() - 1);
    cout << getSolverName() << ": Iterations = " << iteration_stats.size() << ", "
         << "colliding pairs = " << num_of_colliding_pairs << ", "
         << "solution cost = " << sum_of_costs << ", "
         << "initial solution cost = " << initial_sum_of_costs << ", "
         << "runtime = " << runtime << ", "
         << "group size = " << average_group_size << ", "
         << "failed iterations = " << num_of_failures << endl;
    return (num_of_colliding_pairs == 0);
}


bool InitLNS::getInitialSolution()
{
    neighbor.agents.resize(agents.size());
    for (int i = 0; i < (int)agents.size(); i++)
        neighbor.agents[i] = i;
    neighbor.old_sum_of_costs = MAX_COST;
    neighbor.sum_of_costs = 0;
    bool succ = false;
    if (init_algo_name == "PP")
        succ = runPP();
    else
    {
        cerr <<  "Initial MAPF solver " << init_algo_name << " does not exist!" << endl;
        exit(-1);
    }

    initial_sum_of_costs = neighbor.sum_of_costs;
    sum_of_costs = neighbor.sum_of_costs;
    num_of_colliding_pairs = (int) neighbor.colliding_pairs.size();
    for(const auto& agent_pair : neighbor.colliding_pairs)
    {
        collision_graph[agent_pair.first].emplace(agent_pair.second);
        collision_graph[agent_pair.second].emplace(agent_pair.first);
    }
    if (screen >= 2)
        printCollisionGraph();
    return succ;
}

bool InitLNS::runPP()
{
    auto shuffled_agents = neighbor.agents;
    std::random_shuffle(shuffled_agents.begin(), shuffled_agents.end());
    if (screen >= 2) {
        for (auto id : shuffled_agents)
            cout << id << ", ";
        cout << endl;
    }
    int remaining_agents = (int)shuffled_agents.size();
    auto p = shuffled_agents.begin();
    neighbor.sum_of_costs = 0;
    neighbor.colliding_pairs.clear();
    runtime = ((fsec)(Time::now() - start_time)).count();
    double T = time_limit - runtime; // time limit
    if (!iteration_stats.empty()) // replan
        T = min(T, replan_time_limit);
    auto time = Time::now();
    while (p != shuffled_agents.end() && ((fsec)(Time::now() - time)).count() < T)
    {
        int id = *p;
        agents[id].path = agents[id].path_planner.findOptimalPath(path_table);
        assert(!agents[id].path.empty() && agents[id].path.back().location == agents[id].path_planner.goal_location);
        updateCollidingPairs(neighbor.colliding_pairs, agents[id].id, agents[id].path);
        neighbor.sum_of_costs += (int)agents[id].path.size() - 1;
        remaining_agents--;
        if (screen >= 3)
        {
            runtime = ((fsec)(Time::now() - start_time)).count();
            cout << "After agent " << id << ": Remaining agents = " << remaining_agents <<
                 ", colliding pairs = " << neighbor.colliding_pairs.size() <<
                 ", remaining time = " << time_limit - runtime << " seconds. " << endl;
        }
        if (!neighbor.old_colliding_pairs.empty() && // otherwise it is for the first run
            neighbor.colliding_pairs.size() > neighbor.old_colliding_pairs.size())
            break;
        path_table.insertPath(agents[id].id, agents[id].path);
        ++p;
    }
    if (neighbor.old_colliding_pairs.empty()) // first run
    {
        return (p == shuffled_agents.end() && neighbor.colliding_pairs.empty());
    }
    if (p == shuffled_agents.end() && neighbor.colliding_pairs.size() <= neighbor.old_colliding_pairs.size()) // accept new paths
    {
        return true;
    }
    else // stick to old paths
    {
        if (p != shuffled_agents.end())
            num_of_failures++;
        auto p2 = shuffled_agents.begin();
        while (p2 != p)
        {
            int a = *p2;
            path_table.deletePath(agents[a].id, agents[a].path);
            ++p2;
        }
        if (!neighbor.old_paths.empty())
        {
            p2 = neighbor.agents.begin();
            for (int i = 0; i < (int)neighbor.agents.size(); i++)
            {
                int a = *p2;
                agents[a].path = neighbor.old_paths[i];
                path_table.insertPath(agents[a].id, agents[a].path);
                ++p2;
            }
            neighbor.sum_of_costs = neighbor.old_sum_of_costs;
        }
        return false;
    }
}

void InitLNS::updateCollidingPairs(set<pair<int, int>>& colliding_pairs, int agent_id, const Path& path) const
{
    if (path.size() < 2)
        return;
    for (int t = 1; t < (int)path.size(); t++)
    {
        int from = path[t - 1].location;
        int to = path[t].location;
        if ((int)path_table.table[to].size() > t) // vertex conflicts
        {
            for (auto id : path_table.table[to][t])
                colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
        }
        if (from != to && path_table.table[to].size() >= t && path_table.table[from].size() > t) // edge conflicts
        {
            for (auto a1 : path_table.table[to][t - 1])
            {
                for (auto a2: path_table.table[from][t])
                {
                    if (a1 == a2)
                        colliding_pairs.emplace(min(agent_id, a1), max(agent_id, a1));
                }
            }
        }
        if (!path_table.goals.empty() && path_table.goals[to] < t) // target conflicts
        { // this agent traverses the target of another agent
            for (auto id : path_table.table[to][path_table.goals[to]]) // look at all agents at the goal time
            {
                if (agents[id].path.back().location == to) // if agent id's goal is to, then this is the agent we want
                {
                    colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
                    break;
                }
            }
        }
    }
    int goal = path.back().location; // target conflicts - some other agent traverses the target of this agent
    for (int t = (int)path.size(); t < path_table.table[goal].size(); t++)
    {
        for (auto id : path_table.table[goal][t])
            colliding_pairs.emplace(min(agent_id, id), max(agent_id, id));
    }
}

void InitLNS::chooseDestroyHeuristicbyALNS()
{
    double sum = 0;
    for (const auto& h : destroy_weights)
        sum += h;
    if (screen >= 2)
    {
        cout << "destroy weights = ";
        for (const auto& h : destroy_weights)
            cout << h / sum << ",";
    }
    double r = (double) rand() / RAND_MAX;
    double threshold = destroy_weights[0];
    selected_neighbor = 0;
    while (threshold < r * sum)
    {
        selected_neighbor++;
        threshold += destroy_weights[selected_neighbor];
    }
    switch (selected_neighbor / num_neighbor_sizes)
    {
        case 0 : destroy_strategy = RANDOMWALK; break;
        case 1 : destroy_strategy = INTERSECTION; break;
        case 2 : destroy_strategy = RANDOMAGENTS; break;
        default : cerr << "ERROR" << endl; exit(-1);
    }
    // neighbor_size = (int) pow(2, selected_neighbor % num_neighbor_sizes + 1);
}

bool InitLNS::generateNeighborByCollisionGraph()
{
    unordered_map<int, list<int>> G;
    for (int i = 0; i < (int)collision_graph.size(); i++)
    {
        if (!collision_graph[i].empty())
            G[i].assign(collision_graph[i].begin(), collision_graph[i].end());
    }
    assert(!G.empty());
    assert(neighbor_size <= (int)agents.size());
    set<int> neighbors_set;
    if ((int)G.size() < neighbor_size)
    {
        for (const auto& node : G)
            neighbors_set.insert(node.first);
        int count = 0;
        while ((int)neighbors_set.size() < neighbor_size && count < 10)
        {
            int a1 = *std::next(neighbors_set.begin(), rand() % neighbors_set.size());
            int a2 = randomWalk(a1);
            if (a2 != NO_AGENT)
                neighbors_set.insert(a2);
            else
                count++;
        }
    }
    else
    {
        int a = -1;
        while ((int)neighbors_set.size() < neighbor_size)
        {
            if (a == -1)
            {
                a = std::next(G.begin(), rand() % G.size())->first;
                neighbors_set.insert(a);
            }
            else
            {
                a = *std::next(G[a].begin(), rand() % G[a].size());
                auto ret = neighbors_set.insert(a);
                if (!ret.second) // no new element inserted
                    a = -1;
            }
        }
    }
    neighbor.agents.assign(neighbors_set.begin(), neighbors_set.end());
    if (screen >= 2)
        cout << "Generate " << neighbor.agents.size() << " neighbors by collision graph" << endl;
    return true;
}

// Random walk; return the first agent that the agent collides with
int InitLNS::randomWalk(int agent_id)
{
    int t = rand() % agents[agent_id].path.size();
    int loc = agents[agent_id].path[t].location;
    while (t <= path_table.makespan and
           (path_table.table[loc].size() <= t or
           path_table.table[loc][t].empty() or
           (path_table.table[loc][t].size() == 1 and path_table.table[loc][t].front() == agent_id)))
    {
        auto next_locs = instance.getNeighbors(loc);
        next_locs.push_back(loc);
        int step = rand() % next_locs.size();
        auto it = next_locs.begin();
        loc = *std::next(next_locs.begin(), rand() % next_locs.size());
        t = t + 1;
    }
    if (t > path_table.makespan)
        return NO_AGENT;
    else
        return *std::next(path_table.table[loc][t].begin(), rand() % path_table.table[loc][t].size());
}

void InitLNS::validateSolution() const
{
    int sum = 0;
    for (const auto& a1_ : agents)
    {
        if (a1_.path.empty())
        {
            cerr << "No solution for agent " << a1_.id << endl;
            exit(-1);
        }
        else if (a1_.path_planner.start_location != a1_.path.front().location)
        {
            cerr << "The path of agent " << a1_.id << " starts from location " << a1_.path.front().location
                << ", which is different from its start location " << a1_.path_planner.start_location << endl;
            exit(-1);
        }
        else if (a1_.path_planner.goal_location != a1_.path.back().location)
        {
            cerr << "The path of agent " << a1_.id << " ends at location " << a1_.path.back().location
                 << ", which is different from its goal location " << a1_.path_planner.goal_location << endl;
            exit(-1);
        }
        for (int t = 1; t < (int) a1_.path.size(); t++ )
        {
            if (!instance.validMove(a1_.path[t - 1].location, a1_.path[t].location))
            {
                cerr << "The path of agent " << a1_.id << " jump from "
                     << a1_.path[t - 1].location << " to " << a1_.path[t].location
                     << " between timesteps " << t - 1 << " and " << t << endl;
                exit(-1);
            }
        }
        sum += (int) a1_.path.size() - 1;
        for (const auto& a2_: agents)
        {
            if (a1_.id >= a2_.id || a1_.path.empty())
                continue;
            const auto a1 = a1_.path.size() <= a2_.path.size()? a1_ : a2_;
            const auto a2 = a1_.path.size() <= a2_.path.size()? a2_ : a1_;
            int t = 1;
            for (; t < (int) a1.path.size(); t++)
            {
                if (a1.path[t].location == a2.path[t].location) // vertex conflict
                {
                    cerr << "Find a vertex conflict between agents " << a1.id << " and " << a2.id <<
                            " at location " << a1.path[t].location << " at timestep " << t << endl;
                    exit(-1);
                }
                else if (a1.path[t].location == a2.path[t - 1].location &&
                        a1.path[t - 1].location == a2.path[t].location) // edge conflict
                {
                    cerr << "Find an edge conflict between agents " << a1.id << " and " << a2.id <<
                         " at edge (" << a1.path[t - 1].location << "," << a1.path[t].location <<
                         ") at timestep " << t << endl;
                    exit(-1);
                }
            }
            int target = a1.path.back().location;
            for (; t < (int) a2.path.size(); t++)
            {
                if (a2.path[t].location == target)  // target conflict
                {
                    cerr << "Find a target conflict where agent " << a2.id << " (of length " << a2.path.size() - 1<<
                         ") traverses agent " << a1.id << " (of length " << a1.path.size() - 1<<
                         ")'s target location " << target << " at timestep " << t << endl;
                    exit(-1);
                }
            }
        }
    }
    if (sum_of_costs != sum)
    {
        cerr << "The computed sum of costs " << sum_of_costs <<
             " is different from the sum of the paths in the solution " << sum << endl;
        exit(-1);
    }
}

void InitLNS::writeIterStatsToFile(string file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    output << "num of agents," <<
           "sum of costs," <<
           "num of colliding pairs," <<
           "runtime," <<
           "cost lowerbound," <<
           "sum of distances," <<
           "MAPF algorithm" << endl;

    for (const auto &data : iteration_stats)
    {
        output << data.num_of_agents << "," <<
               data.sum_of_costs << "," <<
               data.num_of_colliding_pairs << "," <<
               data.runtime << "," <<
               max(sum_of_costs_lowerbound, sum_of_distances) << "," <<
               sum_of_distances << "," <<
               data.algorithm << endl;
    }
    output.close();
}

void InitLNS::writeResultToFile(string file_name) const
{
    std::ifstream infile(file_name);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        ofstream addHeads(file_name);
        addHeads << "runtime,solution cost,initial solution cost,min f value,root g value," <<
                 "iterations," <<
                 "group size," <<
                 "runtime of initial solution,area under curve," <<
                 "solver name,instance name" << endl;
        addHeads.close();
    }
    ofstream stats(file_name, std::ios::app);
    double auc = 0;
    if (!iteration_stats.empty())
    {
        auto prev = iteration_stats.begin();
        auto curr = prev;
        ++curr;
        while (curr != iteration_stats.end() && curr->runtime < time_limit)
        {
            auc += (prev->sum_of_costs - sum_of_distances) * (curr->runtime - prev->runtime);
            prev = curr;
            ++curr;
        }
        auc += (prev->sum_of_costs - sum_of_distances) * (time_limit - prev->runtime);
    }
    stats << runtime << "," << sum_of_costs << "," << initial_sum_of_costs << "," <<
            max(sum_of_distances, sum_of_costs_lowerbound) << "," << sum_of_distances << "," <<
            iteration_stats.size() << "," << average_group_size << "," <<
            initial_solution_runtime << "," << auc << "," <<
            getSolverName() << "," << instance.getInstanceName() << endl;
    stats.close();
}

void InitLNS::writePathsToFile(string file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    // output << agents.size() << endl;

    for (const auto &agent : agents)
    {
        output << "Agent " << agent.id << ":";
        for (const auto &state : agent.path)
            output << "(" << instance.getRowCoordinate(state.location) << "," <<
                            instance.getColCoordinate(state.location) << ")->";
        output << endl;
    }
    output.close();
}

void InitLNS::printCollisionGraph() const
{
    cout << "Collision graph: ";
    int edges = 0;
    for (size_t i = 0; i < collision_graph.size(); i++)
    {
        for (int j : collision_graph[i])
        {
            if (i < j)
            {
                cout << "(" << i << "," << j << "),";
                edges++;
            }
        }
    }
    cout << endl <<  "|V|=" << collision_graph.size() << ", |E|=" << edges << endl;
}
