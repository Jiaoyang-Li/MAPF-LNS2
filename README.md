# MAPF-LNS2

MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search


MAPF-LNS2 is an efficient algorithm for solving Multi-Agent Path Finding (MAPF). 

The code requires the external libraries 
BOOST (https://www.boost.org/) and Eigen (https://eigen.tuxfamily.org/). 
After you installed both libraries and downloaded the source code, 
go into the directory of the source code and compile it with CMake: 
```
cmake .
make
```

Then, you are able to run the code:
```
./lns -m random-32-32-20.map -a random-32-32-20-random-1.scen -o test.csv -k 400 -t 300
```

- m: the map file from the MAPF benchmark
- a: the scenario file from the MAPF benchmark
- o: the output file
- k: the number of agents
- t: the runtime limit

You can find more details and explanations for all parameters with:
```
./lns --help
```

We provide example instance files "random-32-32-20.map" and "random-32-32-20-random-1.scen" in the repo, 
but more instances can be download from the MAPF benchmark (https://movingai.com/benchmarks/mapf/index.html).
All the experiments in the paper used in instances from the benchmark except for Experiment 5, 
for which the instances are in folder "instances".

## Credits
## Credits

The software was developed by Jiaoyang Li and Zhe Chen based on [MAPF-LNS](https://github.com/Jiaoyang-Li/MAPF-LNS).

The rule-based MAPF solvers (i.e., PPS, PIBT, and winPIBT) inside the software were borrowed from 
https://github.com/Kei18/pibt/tree/v1.3

MAPF-LNS2 is released under USC – Research License. See license.txt for further details.
 
## References
[1] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey and Sven Koenig.
MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search
In Proceedings of the AAAI Conference on Artificial Intelligence, (in print), 2022.