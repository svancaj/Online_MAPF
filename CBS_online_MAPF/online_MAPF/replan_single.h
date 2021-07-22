#include "a_star.h"

#ifndef RS_H
#define RS_H

class ReplanSingle
{
public:
	ReplanSingle(Instance*);

	int Solve();

private:
	Instance* instance;

	std::string statistic_file;
	std::string algorithm;

	std::vector<std::vector<int> > current_plan;	//[agent][timestep] = node (-1 if not in the graph, -2 if in garage)
	std::vector<long long> replan_times;			//replan time for each replan in ms
	int timeouted_calculations;						// number of times we used A-Star calculation instead of desired algorithm
};

#endif /* RS_H */