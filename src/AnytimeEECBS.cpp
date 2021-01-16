#include "AnytimeEECBS.h"
#include "ECBS.h"


void AnytimeEECBS::run()
{
    int num_of_agents = instance.getDefaultNumberOfAgents();
    bool improvements = true;
    ECBS ecbs(instance, false, screen);
    ecbs.setPrioritizeConflicts(true);
    ecbs.setDisjointSplitting(false);
    ecbs.setBypass(true);
    ecbs.setRectangleReasoning(true);
    ecbs.setCorridorReasoning(true);
    ecbs.setHeuristicType(heuristics_type::WDG, heuristics_type::GLOBAL);
    ecbs.setTargetReasoning(true);
    ecbs.setMutexReasoning(false);
    ecbs.setConflictSelectionRule(conflict_selection::EARLIEST);
    ecbs.setNodeSelectionRule(node_selection::NODE_CONFLICTPAIRS);
    ecbs.setSavingStats(false);
    preprocessing_time = ecbs.runtime_preprocessing;
    sum_of_distances = 0;
    for (int i = 0; i < num_of_agents; i++)
    {
        sum_of_distances += ecbs.getSearchEngine(i)->my_heuristic[ecbs.getSearchEngine(i)->start_location];
    }

    // run
    double w = 2;
    while(runtime < time_limit && sum_of_costs > sum_of_costs_lowerbound)
    {
        ecbs.clear();
        ecbs.setHighLevelSolver(high_level_solver_type::EES, w);
        ecbs.solve(time_limit - runtime, sum_of_costs_lowerbound);
        runtime += ecbs.runtime;
        assert(sum_of_costs_lowerbound <= ecbs.getLowerBound());
        sum_of_costs_lowerbound = ecbs.getLowerBound();
        if (ecbs.solution_found)
        {
            if (sum_of_costs > ecbs.solution_cost)
            {
                sum_of_costs = ecbs.solution_cost;
                solution.resize(num_of_agents);
                for (int i = 0; i < num_of_agents; i++)
                    solution[i] = *ecbs.paths[i];
            }
            // w = max(1.0, 0.99 * sum_of_costs / sum_of_costs_lowerbound);
            // a better way of computing w should be
            w = 1 + 0.99 * (sum_of_costs * 1.0 / sum_of_costs_lowerbound - 1);
            iteration_stats.emplace_back(instance.getDefaultNumberOfAgents(), sum_of_costs,
                                         runtime, "EECBS("+ std::to_string(w) + ")", sum_of_costs_lowerbound);
        }

        if (screen >= 1)
            cout << "Iteration " << iteration_stats.size() << ", "
                 << "lower bound = " << sum_of_costs_lowerbound << ", "
                 << "solution cost = " << sum_of_costs << ", "
                 << "new w = " << w << ", "
                 << "remaining time = " << time_limit - runtime << endl;
    }
    ecbs.clearSearchEngines();
    cout << getSolverName() << ": Iterations = " << iteration_stats.size() << ", "
         << "lower bound = " << sum_of_costs_lowerbound << ", "
         << "solution cost = " << sum_of_costs << ", "
         << "initial solution cost = " << iteration_stats.front().sum_of_costs << ", "
         << "runtime = " << runtime << endl;
}


void AnytimeEECBS::validateSolution() const
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

void AnytimeEECBS::writeIterStatsToFile(string file_name) const
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

void AnytimeEECBS::writeResultToFile(string file_name) const
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
          iteration_stats.size() << "," << iteration_stats.front().runtime << "," << auc << "," <<
          preprocessing_time << "," << getSolverName() << "," << instance.getInstanceName() << endl;
    stats.close();
}

