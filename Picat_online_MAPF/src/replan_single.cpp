#include "replan_single.h"

using namespace std;

typedef unordered_multimap<int,State*> MyMap;

ReplanSingle::ReplanSingle(Instance* inst)
{
	instance = inst;
	statistic_file = "results_replan_single.st";
	algorithm = "RS";
	timeouted_calculations = 0;
}

int ReplanSingle::Solve()
{
	for (int i = 0; i < instance->new_agents;)
	{
		int time = instance->appear[i];

		vector<int> current_agents;
		while (time == instance->appear[i])
		{
			current_agents.push_back(i);
			i++;
		}

		AStar AS(instance);
		AS.Solve(current_plan, current_agents);

		long long runtime = AS.current_runtime;
		replan_times.push_back(runtime);
		
		if (runtime > instance->timeout * 1000)
		{
			instance->PrintStatistic(statistic_file, algorithm, instance->ComputeSoC(current_plan), 0, true, replan_times, timeouted_calculations);
			return -1;
		}

		current_plan = AS.current_plan;
		timeouted_calculations++;

		instance->PrintPlan(current_plan);
		cout << endl;
	}
	instance->PrintStatistic(statistic_file, algorithm, instance->ComputeSoC(current_plan), 0, false, replan_times, timeouted_calculations);

	instance->PrintPlan(current_plan);
	instance->CheckPlan(current_plan);
	
	return 0;
}
