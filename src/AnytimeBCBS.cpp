#include "AnytimeBCBS.h"
#include "CBS.h"


void AnytimeBCBS::run()
{
    int num_of_agents = instance.getDefaultNumberOfAgents();
    bool improvements = true;
    double w = (double) MAX_COST / (instance.num_of_cols + instance.num_of_rows)
            / 10 / num_of_agents; // a large enough w without integer overflow issue
    CBS bcbs(instance, false, screen);
    bcbs.setPrioritizeConflicts(improvements);
    bcbs.setDisjointSplitting(false);
    bcbs.setBypass(improvements);
    bcbs.setRectangleReasoning(improvements);
    bcbs.setCorridorReasoning(improvements);
    bcbs.setHeuristicType(improvements? heuristics_type::WDG : heuristics_type::ZERO, heuristics_type::ZERO);
    bcbs.setTargetReasoning(improvements);
    bcbs.setMutexReasoning(false);
    bcbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    bcbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    bcbs.setSavingStats(false);
    bcbs.setHighLevelSolver(high_level_solver_type::ASTAREPS, w);

    preprocessing_time = bcbs.runtime_preprocessing;
    sum_of_distances = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
        sum_of_distances += bcbs.getSearchEngine(i)->my_heuristic[bcbs.getSearchEngine(i)->start_location];
    }

    // run
    CBSNode* best_goal_node = nullptr;
    while(runtime < time_limit && sum_of_costs > sum_of_costs_lowerbound)
    {
        bcbs.solve(time_limit - runtime, sum_of_costs_lowerbound, sum_of_costs);
        runtime += bcbs.runtime;
        assert(sum_of_costs_lowerbound <= bcbs.getLowerBound());
        sum_of_costs_lowerbound = bcbs.getLowerBound();
        if (bcbs.solution_found)
        {
            assert(sum_of_costs > bcbs.solution_cost);
            sum_of_costs = bcbs.solution_cost;
            best_goal_node = bcbs.getGoalNode();
            iteration_stats.emplace_back(instance.getDefaultNumberOfAgents(), sum_of_costs,
                                         runtime, "BCBS", sum_of_costs_lowerbound);
        }

        if (screen >= 1)
            cout << "Iteration " << iteration_stats.size() << ", "
                 << "lower bound = " << sum_of_costs_lowerbound << ", "
                 << "solution cost = " << sum_of_costs << ", "
                 << "remaining time = " << time_limit - runtime << endl;
    }
    if (best_goal_node == nullptr)
        sum_of_costs = -1; // no solution found
    else
    {
        solution.resize(num_of_agents);
        bcbs.updatePaths(best_goal_node);
        for (int i = 0; i < num_of_agents; i++)
            solution[i] = *bcbs.paths[i];
    }
    bcbs.clearSearchEngines();
    cout << getSolverName() << ": Iterations = " << iteration_stats.size() << ", "
         << "lower bound = " << sum_of_costs_lowerbound << ", "
         << "solution cost = " << sum_of_costs << ", "
         << "initial solution cost = " << iteration_stats.front().sum_of_costs << ", "
         << "runtime = " << runtime << endl;
}


void AnytimeBCBS::validateSolution() const
{
    if (solution.empty())
        return;
    int N = instance.getDefaultNumberOfAgents();
    for (int i = 0; i < N; i++)
    {
        for (int j = i + 1; j < N; j++)
        {
            const auto a1 = solution[i].size() <= solution[j].size()? i : j;
            const auto a2 = solution[i].size() <= solution[j].size()? j : i;
            int t = 1;
            for (; t < (int) solution[a1].size(); t++)
            {
                if (solution[a1][t].location == solution[a2][t].location) // vertex conflict
                {
                    cerr << "Find a vertex conflict between agents " << a1 << " and " << a2 <<
                         " at location " << solution[a1][t].location << " at timestep " << t << endl;
                    exit(-1);
                }
                else if (solution[a1][t].location == solution[a2][t - 1].location &&
                        solution[a1][t - 1].location == solution[a2][t].location) // edge conflict
                {
                    cerr << "Find an edge conflict between agents " << a1 << " and " << a2 <<
                         " at edge (" << solution[a1][t - 1].location << "," << solution[a1][t].location <<
                         ") at timestep " << t << endl;
                    exit(-1);
                }
            }
            int target = solution[a1].back().location;
            for (; t < (int) solution[a2].size(); t++)
            {
                if (solution[a2][t].location == target)  // target conflict
                {
                    cerr << "Find a target conflict where agent " << a2 << " traverses agent " << a1 <<
                         "'s target location " << target << " at timestep " << t << endl;
                    exit(-1);
                }
            }
        }
    }
}

void AnytimeBCBS::writeIterStatsToFile(string file_name) const
{
    std::ofstream output;
    output.open(file_name);
    // header
    output << "num of agents," <<
           "sum of costs," <<
           "runtime," <<
           "cost lowerbound," <<
           "MAPF algorithm" << endl;

    for (const auto &data : iteration_stats)
    {
        output << data.num_of_agents << "," <<
               data.sum_of_costs << "," <<
               data.runtime << "," <<
               data.sum_of_costs_lowerbound << "," <<
               data.algorithm << endl;
    }
    output.close();
}

void AnytimeBCBS::writeResultToFile(string file_name) const
{
    std::ifstream infile(file_name);
    bool exist = infile.good();
    infile.close();
    if (!exist)
    {
        ofstream addHeads(file_name);
        addHeads << "runtime,solution cost,initial solution cost,min f value,root g value," <<
                 "iterations," <<
                 "runtime of initial solution,area under curve," <<
                 "preprocessing runtime,solver name,instance name" << endl;
        addHeads.close();
    }
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
    ofstream stats(file_name, std::ios::app);
    stats << runtime << "," << sum_of_costs << "," << iteration_stats.front().sum_of_costs << "," <<
          sum_of_costs_lowerbound << "," << sum_of_distances << "," <<
          iteration_stats.size() << "," << iteration_stats.front().runtime << "," <<  auc << "," <<
          preprocessing_time << "," << getSolverName() << "," << instance.getInstanceName() << endl;
    stats.close();
}
