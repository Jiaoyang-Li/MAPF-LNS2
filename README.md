# LNS

Anytime Multi-Agent Path Finding via Large Neighborhood Search


MAPF-LNS is an effifent anytime algorithm for solving Multi-Agent Path Finding (MAPF). 
More details can be found in our paper at AAMAS 2021 [1].

The code requires the external libraries 
BOOST (https://www.boost.org/) and Eigen (https://eigen.tuxfamily.org/). 
After you installed both libraries and downloaded the source code, 
go into the directory of the source code and compile it with CMake: 
```
cmake .
make
```

You also need to download the MAPF instances from the MAPF benchmark (https://movingai.com/benchmarks/mapf/index.html).

Then, you are able to run the code:
```
./lns -m random-32-32-20.map -a random-32-32-20-random-1.scen -o test.csv -k 50 -t 60
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

## Credits

The software was developed by Jiaoyang Li and Zhe Chen.

The rule-based MAPF solvers (i.e., PPS, PIBT, and winPIBT) inside the software were borrowed from 
https://github.com/Kei18/pibt/tree/v1.3

MAPF-LNS is released under USC – Research License. See license.txt for further details.
 
## References
[1] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey, Sven Koenig.
Anytime Multi-Agent Path Finding via Large Neighborhood Search: Extended Abstract.
In Proceedings of the International Conference on Autonomous Agents and Multiagent Systems (AAMAS), (in print), 2021.

 

 
