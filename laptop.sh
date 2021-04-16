#!/bin/sh

timelimit=10
name=random-32-32-20
map=../../MAPF-instances/mapf-map/$name.map
scen=../../MAPF-instances/mapf-scen-random/scen-random/$name-random
output=$name-random

for k in $(seq 100 50 250)
do
  for i in $(seq 1 1 25)
  do
    echo $k agents on instance $name-random-$i
    ./lns -m $map -a $scen-$i.scen -k $k -t $timelimit -s 0 --solver=LNS --initAlgo=PP --replanAlgo=PP --neighborSize=4
  done
done

for k in $(seq 100 50 250)
do
  for i in $(seq 1 1 25)
  do
    echo $k agents on instance $name-random-$i
    ./lns -m $map -a $scen-$i.scen -k $k -t $timelimit -s 0 --solver=LNS --initAlgo=PP --replanAlgo=PP --neighborSize=8
  done
done

for k in $(seq 100 50 250)
do
  for i in $(seq 1 1 25)
  do
    echo $k agents on instance $name-random-$i
    ./lns -m $map -a $scen-$i.scen -k $k -t $timelimit -s 0 --solver=LNS --initAlgo=PP --replanAlgo=PP --neighborSize=16
  done
done

for k in $(seq 100 50 250)
do
  for i in $(seq 1 1 25)
  do
    echo $k agents on instance $name-random-$i
    ./lns -m $map -a $scen-$i.scen -k $k -t $timelimit -s 0 --solver=LNS --initAlgo=PP --replanAlgo=PP --neighborSize=32
  done
done