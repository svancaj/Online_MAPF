#include "a_star.h"

#ifndef ID_H
#define ID_H

class IndependenceDetection
{
public:
	IndependenceDetection(Instance*, float);

	int Solve();

private:
	bool CheckForConflicts(int, int&, int&);
	bool CheckPastConflicts(std::vector<std::pair<int, int> >&, int, int);
	void MergeGroups(int, int);
	int ComputeGroupSOC(int, int, int);
	void ComputeNubmerOfRerouts(int, std::vector<std::vector<int> >&);
	int PlanForGroups(int, int, int, int);

	Instance* instance;
	float suboptimality;

	std::string statistic_file;
	std::string algorithm;
	int current_plan_length;
	int actually_replaned_agents;
	long long current_runtime;
	int timeouted_calculations;		// number of times we used A-Star calculation instead of desired algorithm

	std::vector<std::vector<int> > current_plan;	// [agent][timestep] = node (-1 if not in the graph, -2 if in garage)
	std::vector<std::vector<int> > groups;			// list of groups, each group is a list of agents
	std::vector<int> agent_to_group;				// an id of a group for each agent

	std::vector<long long> replan_times;			// replan time for each replan in ms
};

#endif /* ID_H */