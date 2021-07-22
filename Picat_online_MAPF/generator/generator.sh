#!/bin/bash

for type in 10 11 12 13
do
for new_agents in 10 12 15 17 20 22 25
do
for steps in 30
do
	echo "intersection -t $type -a $new_agents -m $steps -x 10"
	./intersection -t $type -a $new_agents -m $steps -x 5
done
done
done

mv instances/* ../instances/
