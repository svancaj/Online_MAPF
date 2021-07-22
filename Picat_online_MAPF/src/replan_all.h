#include "a_star.h"

#ifndef RA_H
#define RA_H

class ReplanAll
{
public:
	ReplanAll(Instance*, int);

	int Solve();

private:
	void PrintFile(int);
	int ReadResults(int, int);
	void FixGoalPosition();

	Instance* instance;

	std::string statistic_file;
	std::string algorithm;
	int fixed_path;					// 0 = replan all; 1 = replan single grouped
	int actually_replaned_agents;	// number of agents that have different path after replan
	int timeouted_calculations;		// number of times we used A-Star calculation instead of desired algorithm
	int current_plan_length;

	std::vector<std::vector<int> > current_plan;	//[agent][timestep] = node (-1 if not in the graph, -2 if in garage)
	std::vector<long long> replan_times;			//replan time for each replan in ms
};

#endif /* RA_H */