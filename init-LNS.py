#!/usr/bin/env python3
import os
import numpy as np
import csv
import matplotlib.pyplot as plt
from pathlib import Path

colors = ['#E74C3C', '#27AE60', '#F1C40F', '#A569BD', '#5DADE2', '#E67E22', '#95A5A6', '#1E8449', '#2980B9', 'k']
markers = ['s-', 'o-', 'd-', '>-', '*-', 'p-', 'v-', '<-', 'h', 'x-']
linestyle = ['--', '-', ':', '-.']
fontsize = 10
labelsize = 18

time_limit = 10
folder = 'D:/LNS/experiments/initLNS/GCBS/' # 'cmake-build-release/'
instance_name = 'random-32-32-20-random-'  # 'warehouse-10-20-10-2-1-random-150'  #
algorithms = ["neighbor=4", "neighbor=8", "neighbor=16", "neighbor=32"]
algo_names = ["neighbor=4", "neighbor=8", "neighbor=16", "neighbor=32"]

instance_ids = [i for i in range(1, 26)]
agents = [i for i in range(100, 350, 50)]
rsts = {}
for agent in agents:
    rst = {}
    for instance_id in instance_ids:
        if instance_id not in rst:
            rst[instance_id] = {}
        for (algorithm, algo_name) in zip(algorithms, algo_names):
            file_name = folder + instance_name + str(instance_id) + '.scen-agents=' + str(agent) + '-' + algorithm + '-initLNS.csv'
            f = Path(file_name)
            if not f.is_file():
                print(file_name + " does not exist.")
                continue
            with open(file_name, newline='') as csvfile:
                reader = csv.DictReader(csvfile, delimiter=',', quotechar='|')
                rst[instance_id][algo_name] = {}
                for key in reader.fieldnames:
                    rst[instance_id][algo_name][key] = []
                for row in reader:
                    for key in reader.fieldnames:
                        try:
                            float(row[key])
                            rst[instance_id][algo_name][key].append(float(row[key]))
                        except ValueError:
                            rst[instance_id][algo_name][key].append(row[key])
    rsts[agent] = rst


success_rates = {}
runtimes = {}
collisions = {}
iterations = {}
runtimes['PP'] = {}
collisions['PP'] = {}
iterations['PP'] = {}
for agent in agents:
    runtime = []
    collision = []
    iteration = []
    for instance_id in instance_ids:
        collision.append(rsts[agent][instance_id][algo_names[0]]['num of colliding pairs'][0])
        if collision[-1] == 0:
            runtime.append(rsts[agent][instance_id][algorithm]['runtime'][0])
        else:
            runtime.append(time_limit)
        iteration.append(len(rsts[agent][instance_id][algo_names[0]]['num of colliding pairs']) - 1)
    runtimes['PP'][agent] = runtime
    collisions['PP'][agent] = collision
    iterations['PP'][agent] = iteration
print('PP')
for agent in agents:
    print('{} agents: success={}, runtime={}, collisions={}'.format(agent,
                                                     sum(1 if t < time_limit else 0 for t in
                                                         runtimes['PP'][agent]) / len(runtimes['PP'][agent]),
                                                     np.mean(runtimes['PP'][agent]), np.mean(collisions['PP'][agent])))
for algo_name in algo_names:
    runtimes[algo_name] = {}
    collisions[algo_name] = {}
    iterations[algo_name] = {}
    for agent in agents:
        runtime = []
        collision = []
        iteration = []
        for instance_id in instance_ids:
            collision.append(rsts[agent][instance_id][algo_name]['num of colliding pairs'][-1])
            if collision[-1] == 0:
                runtime.append(rsts[agent][instance_id][algorithm]['runtime'][-1])
            else:
                runtime.append(time_limit)
            iteration.append(len(rsts[agent][instance_id][algo_name]['num of colliding pairs']) - 1)
        runtimes[algo_name][agent] = runtime
        collisions[algo_name][agent] = collision
        iterations[algo_name][agent] = iteration

    print(algo_name)
    for agent in agents:
        print('{} agents: success={}, runtime={}, collisions={}, iterations={}'.format(agent,
            sum(1 if t < time_limit else 0 for t in runtimes[algo_name][agent]) / len(runtimes[algo_name][agent]),
            np.mean(runtimes[algo_name][agent]), np.mean(collisions[algo_name][agent]),
            np.mean(iterations[algo_name][agent])))

for agent in agents:
    rst = rsts[agent]
    for instance_id in rst:
        print(instance_id)
        plt.figure()
        #plt.subplot(1, 2, 1)
        #plt.title(instance_id, fontsize=fontsize)
        plt.tick_params(labelsize=labelsize)
        fig = plt.gcf()
        fig.set_size_inches(5, 4)
        for i, algorithm in enumerate(algo_names):
            if algorithm not in rst[instance_id]:
                continue
            if len(rst[instance_id][algorithm]['sum of costs']) > 0:
                x = rst[instance_id][algorithm]['runtime']  # + [time_limit]
                y = [rst[instance_id][algorithm]['num of colliding pairs'][i]
                     for i in range(len(rst[instance_id][algorithm]['num of colliding pairs']))]
                # y = y + [y[-1]]
                plt.plot(x, y, markers[i % len(markers)], color=colors[i % len(colors)], markersize=2, linewidth=1,
                         label=algorithm)  # + "-" + str(len(rst[instance_id][algorithm]['runtime'])) + "iters")
        plt.hlines(0, 0, 10, linestyle='dashed', colors=[0,0,0])
        plt.xlabel('Runtime [sec]', fontsize=fontsize)
        plt.ylabel('Number of colliding pairs', fontsize=fontsize)
        plt.legend(fontsize=fontsize)  # loc='center left', bbox_to_anchor=(1, 0.5),  , ncol=len(self.all_algorithm) + 3)
        plt.title(str(agent)+ ' agents on Instance ' + str(instance_id))
        #plt.xscale('log')
        #plt.legend(frameon=True, fontsize=fontsize)

        plt.savefig("analysis/figures/init-LNS/init_LNS_agents_" + str(agent) + "_instance_" + str(instance_id), bbox_inches='tight')
        plt.close()
        # plt.show()