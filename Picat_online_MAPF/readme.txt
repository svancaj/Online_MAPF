online MAPF solver using Picat

Important files:

makefile		- make all = compile the solver
				- make run = solves all instances

src/*			- source files for the solver

generator/* 	- the generator used to create all instances

instances		- input files

bin/olmapf		- solver binary

bin/soc.pi 		- picat solver

bin/results*	- result files for each algorithm

Results file format:
file name;algorithm name;lowerbound SoC;total SoC;rerouted agents;fail / success;timeouted_calculations;total time;maximal time

file name					= name of the instance computed
algorithm name				= what algorithm we used
lowerbound soc 				= lowerbound on sum of costs (ie. sum of paths for each agent while disregarding other agents)
total soc 					= total sum of cost of the solution found
rerouted agents				= number of agents that had to change its plan during computation
fail / success				= did we find valid solution in time? - or perhaps there was some error
timeouted_calculations		= number of calculations that failed using the "clever" algorithm and had to use replan single as a fallback
total time 					= total time of solving the whole instance
maximal time 				= the maximal time spend when new agent appeared