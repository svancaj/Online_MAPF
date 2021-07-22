#!/bin/bash

for type in 13
do
for new_agents in 15 30 45 60
do
for steps in 30
do
	echo "intersection -t $type -a $new_agents -m $steps -x 10"
	./intersection -t $type -a $new_agents -m $steps -x 5
done
done
done

mv instances/* ../instances/
exit 0

for type in 1 2 3
do
for new_agents in 20 30 40
do
for steps in 30
do
	echo "intersection -t $type -a $new_agents -m $steps -x 10"
	./intersection -t $type -a $new_agents -m $steps -x 5
done
done
done

for type in 4 5 6
do
for new_agents in 30 40 50
do
for steps in 40
do
	echo "intersection -t $type -a $new_agents -m $steps -x 10"
	./intersection -t $type -a $new_agents -m $steps -x 5
done
done
done

for type in 7 8 9
do
for new_agents in 40 50 60
do
for steps in 50
do
	echo "intersection -t $type -a $new_agents -m $steps -x 10"
	./intersection -t $type -a $new_agents -m $steps -x 5
done
done
done

mv instances/* ../instances/
