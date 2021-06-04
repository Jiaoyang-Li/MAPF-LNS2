#!/bin/sh

timelimit=10
name=random-32-32-20
map=../../MAPF-instances/mapf-map/$name.map
scen=../../MAPF-instances/mapf-scen-random/scen-random/$name-random
output=$name-random
for k in $(seq 300 50 300)
do
  for i in $(seq 1 1 25)
  do
    echo $k agents on instance $name-random-$i
    #./lns -m $map -a $scen-$i.scen -k $k -t $timelimit -s 0 --solver=LNS --initAlgo=EECBS --replanAlgo=PP --neighborSize=8  --initDestoryStrategy=Adaptive --initLNS=0 --maxIterations=0
    ./lns -m $map -a $scen-$i.scen -k $k -t $timelimit -s 0 --solver=LNS --initAlgo=PP --replanAlgo=PP --neighborSize=8  --initDestoryStrategy=Adaptive
  done
done
exit

timelimit=60
name=Paris_1_256
map=../../MAPF-instances/mapf-map/$name.map
scen=../../MAPF-instances/mapf-scen-random/scen-random/$name-random
output=$name-random
for k in $(seq 700 50 700)
do
  for i in $(seq 1 1 25)
  do
    echo $k agents on instance $name-random-$i
    ./lns -m $map -a $scen-$i.scen -k $k -t $timelimit -s 0 --solver=LNS --initAlgo=PP --replanAlgo=PP --neighborSize=8  --initDestoryStrategy=Adaptive
    ./lns -m $map -a $scen-$i.scen -k $k -t $timelimit -s 0 --solver=LNS --initAlgo=PP --replanAlgo=PP --neighborSize=8  --initDestoryStrategy=Adaptive --initLNS=0 --maxIterations=0
  done
done
exit





