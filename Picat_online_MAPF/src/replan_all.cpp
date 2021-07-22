#include "replan_all.h"

using namespace std;

ReplanAll::ReplanAll(Instance* inst, int fix)
{
	instance = inst;
	if (fix == 1)
	{
		statistic_file = "results_replan_single_grouped.st";
		algorithm = "RSG";
	}
	else if (fix == 0)
	{
		statistic_file = "results_replan_all.st";
		algorithm = "RA";
	}
	fixed_path = fix;
	actually_replaned_agents = 0;
	current_plan_length = 0;
	timeouted_calculations = 0;
}

int ReplanAll::Solve()
{
	for (int i = 0; i < instance->new_agents; )
	{
		int time = instance->appear[i];
		long long runtime = 0;

		vector<int> current_agents;
		while (time == instance->appear[i])
		{
			current_agents.push_back(i);
			i++;
		}

		AStar AS(instance);
		AS.Solve(current_plan, current_agents);

		runtime += AS.current_runtime;

		if (runtime > instance->timeout * 1000)
		{
			instance->PrintStatistic(statistic_file, algorithm, instance->ComputeSoC(current_plan), actually_replaned_agents, true, replan_times, timeouted_calculations);
			return -1;
		}

		for (size_t j = 0; j < current_agents.size(); j++)
		{
			current_plan.push_back(vector<int>(time + 1, -1));
			current_plan[current_plan.size() - 1][time] = -2;
		}

		PrintFile(time);

		string executable;
		executable = string("timeout ").append(to_string(instance->timeout - runtime/1000)).append(string(" ./picat soc ")).append(instance->solver_input_filename).append(" > ").append(instance->solver_output_filename);
		chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		system(executable.c_str());

		chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

		runtime += std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

		replan_times.push_back(runtime);

		if (runtime > instance->timeout * 1000)
		{
			current_plan = AS.current_plan;
			current_plan_length = AS.current_plan_length;
			timeouted_calculations++;
			continue;
		}

		if (ReadResults(time, current_agents.size()) == -1)
		{
			current_plan = AS.current_plan;
			current_plan_length = AS.current_plan_length;
			timeouted_calculations++;
			continue;
		}

		instance->PrintPlan(current_plan);
		cout << endl;
	}
	instance->PrintStatistic(statistic_file, algorithm, instance->ComputeSoC(current_plan), actually_replaned_agents, false, replan_times, timeouted_calculations);

	instance->PrintPlan(current_plan);
	instance->CheckPlan(current_plan);

	return 0;
}

void ReplanAll::PrintFile(int time)
{
	vector<pair<int,int> > agents;
	vector<pair<int,int> > garages;
	vector<int> to_print(current_plan.size());
	int garage_id = instance->nodes;

	for (size_t i = 0; i < current_plan.size(); i++)
	{
		if ((int)current_plan[i].size() > time)
		{
			if (current_plan[i][time] >= 0)		// in transit
			{
				agents.push_back(make_pair(current_plan[i][time], instance->goal[i]));
				to_print[i] = fixed_path;
			}
			else if (current_plan[i][time] == -2)	// in garage
			{
				agents.push_back(make_pair(garage_id, instance->goal[i]));
				garages.push_back(make_pair(garage_id, instance->start[i]));
				garage_id++;
				to_print[i] = fixed_path;
			}
			else
				to_print[i] = -1;	// not in graph
		}
		else
			to_print[i] = -1;
	}

	instance->PrintAll(time, current_plan, garages, agents, to_print);
	//instance->PrintPlan(current_plan, to_print, time);
}

int ReplanAll::ReadResults(int time, int new_agents)
{
	string line;
	ifstream input(instance->solver_output_filename);
	if (input.is_open())
	{
		while (getline(input, line))
			if (line.compare("agents | timesteps") == 0)
				break;

		if (input.eof())
		{
			input.close();
			return -1; // timeout or other error
		}

		getline(input, line);
		stringstream ss(line);
		int agents, timesteps;
		ss >> agents;
		ss >> timesteps;

		vector<int> agent_to_update;

		for (size_t i = 0; i < current_plan.size(); i++)
		{
			current_plan[i].resize(time + timesteps, -1);
			if (current_plan[i][time] != -1)
				agent_to_update.push_back(i);
		}

		for (int i = 0; i < agents; i++)
		{
			bool changed_path = false;
			getline(input, line);
			stringstream single_agent_plan(line);
			for (int j = 0; j < timesteps; j++)
			{
				int location;
				single_agent_plan >> location;
				if (location > instance->nodes)
				{
					if (current_plan[agent_to_update[i]][time + j] != -2)
						changed_path = true;
					current_plan[agent_to_update[i]][time + j] = -2;
				}
				else
				{
					if (current_plan[agent_to_update[i]][time + j] != location - 1)
						changed_path = true;
					current_plan[agent_to_update[i]][time + j] = location - 1;
				}
			}
			if (changed_path && i < agents - new_agents)
				actually_replaned_agents++;
		}
		input.close();
		//string executable = "del /f ";
		//executable.append(instance->solver_output_filename);
		//system(executable.c_str());
	}
	else
		return -1;

	FixGoalPosition();

	return 0;
}

void ReplanAll::FixGoalPosition()
{
	int max_timestep = 0;

	for (size_t i = 0; i < current_plan.size(); i++)
	{
		for (size_t j = 0; j < current_plan[i].size(); j++)
		{
			if (current_plan[i][j] == instance->goal[i])
			{
				for (size_t k = j + 1; k < current_plan[i].size(); k++)
					current_plan[i][k] = -1;
				if (j > max_timestep)
					max_timestep = j;
				break;
			}
		}
	}

	// trim extra timesteps
	current_plan_length = max_timestep + 1;
	for (size_t i = 0; i < current_plan.size(); i++)
	current_plan[i].resize(current_plan_length);
}
