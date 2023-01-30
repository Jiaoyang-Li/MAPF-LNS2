# MAPF-LNS2
![test_ubuntu](https://github.com/Jiaoyang-Li/MAPF-LNS2/actions/workflows/test_ubuntu.yml/badge.svg)
![test_macos](https://github.com/Jiaoyang-Li/MAPF-LNS2/actions/workflows/test_macos.yml/badge.svg)

MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search


MAPF-LNS2 is an efficient algorithm for solving Multi-Agent Path Finding (MAPF). 
More details can be found in our paper at AAAI 2022 [1].

This software is an advanced version and a superset of MAPF-LNS (https://github.com/Jiaoyang-Li/MAPF-LNS); that is, it also contains Anytime Multi-Agent Path Finding via Large Neighborhood Search [2]. 

## Usage
The code requires the external libraries 
BOOST (https://www.boost.org/) and Eigen (https://eigen.tuxfamily.org/). 
Here is an easy way of installing the required libraries on Ubuntu:    
```shell script
sudo apt update
```
- Install the Eigen library (used for linear algebra computing)
    ```shell script
    sudo apt install libeigen3-dev
    ```
- Install the boost library 
    ```shell script
    sudo apt install libboost-all-dev
    ```
    
After you installed both libraries and downloaded the source code, 
go into the directory of the source code and compile it with CMake: 
```shell script
cmake -DCMAKE_BUILD_TYPE=RELEASE .
make
```

Then, you are able to run the code:
```
./lns -m random-32-32-20.map -a random-32-32-20-random-1.scen -o test -k 400 -t 300 --outputPaths=paths.txt 
```

- m: the map file from the MAPF benchmark
- a: the scenario file from the MAPF benchmark
- o: the output file name (no need for file extension)
- k: the number of agents
- t: the runtime limit
- outputPaths: the output file that contains the paths

You can find more details and explanations for all parameters with:
```
./lns --help
```

We provide example instance files "random-32-32-20.map" and "random-32-32-20-random-1.scen" in the repo. 
More instances can be download from the [MAPF benchmark](https://movingai.com/benchmarks/mapf/index.html).
All the experiments in the paper used in instances from the benchmark except for Experiment 5, 
for which the instances are in folder "instances". 
In particular, the format of the scen files is explained [here](https://movingai.com/benchmarks/formats.html). 
For a given number of agents k, the first k rows of the scen file are used to generate the k pairs of start and target locations.

## Credits

The software was developed by Jiaoyang Li and Zhe Chen based on [MAPF-LNS](https://github.com/Jiaoyang-Li/MAPF-LNS).

The rule-based MAPF solvers (i.e., PPS, PIBT, and winPIBT) inside the software were borrowed from 
https://github.com/Kei18/pibt/tree/v1.3

MAPF-LNS2 is released under USC – Research License. See license.txt for further details.
 
## References
[1] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey and Sven Koenig.
MAPF-LNS2: Fast Repairing for Multi-Agent Path Finding via Large Neighborhood Search
In Proceedings of the AAAI Conference on Artificial Intelligence, (in print), 2022.

[2] Jiaoyang Li, Zhe Chen, Daniel Harabor, Peter J. Stuckey, Sven Koenig. 
Anytime Multi-Agent Path Finding via Large Neighborhood Search. 
In Proceedings of the International Joint Conference on Artificial Intelligence (IJCAI), pages 4127-4135, 2021.
