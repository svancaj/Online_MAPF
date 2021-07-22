#include "independence_detection.h"

using namespace std;

IndependenceDetection::IndependenceDetection(Instance* inst, float sub)
{
	instance = inst;
	suboptimality = sub;
	if (sub == 1)
	{
		statistic_file = "results_ID.st";
		algorithm = "ID";
	}
	else
	{
		statistic_file = "results_subID.st";
		algorithm = "subID";
	}
	actually_replaned_agents = 0;
	current_plan_length = 0;
	timeouted_calculations = 0;
}

int IndependenceDetection::Solve()
{
	for (int i = 0; i < instance->new_agents;)
	{
		int time = instance->appear[i];
		current_runtime = 0;
		int first_new_agent = groups.size();
		int ret_val;

		vector<vector<int> > plan_backup = current_plan;
		vector<int> current_agents;
		vector<pair<int, int> > conflicted_groups;
		
		if (time >= current_plan_length)
		{
			for (size_t j = 0; j < current_plan.size(); j++)
				current_plan[j].resize(time + 1, -1);
			current_plan_length = time + 1;
		}

		while (time == instance->appear[i])
		{
			agent_to_group.push_back(groups.size());
			groups.push_back(vector<int> (1,i));
			current_agents.push_back(i);
			current_plan.push_back(vector<int> (current_plan_length, -1));
			current_plan[i][time] = -2;
			i++;
		}

		// fallback plan
		AStar AS(instance);
		AS.Solve(plan_backup, current_agents);
		current_runtime += AS.current_runtime;
		if (current_runtime > instance->timeout * 1000)
		{
			instance->PrintStatistic(statistic_file, algorithm, instance->ComputeSoC(current_plan), actually_replaned_agents, true, replan_times, timeouted_calculations);
			return -1;
		}
		vector<vector<int> > groups_backup = groups;


		// plan for new groups
		for (int j = first_new_agent; j < groups.size(); j++)
		{
			if (PlanForGroups(time, j, -1, -1) == -100)
			{
				current_plan = AS.current_plan;
				current_plan_length = AS.current_plan_length;
				timeouted_calculations++;
				groups = groups_backup;
				break;
			}
		}

		// find conflicting groups and replan for them
		int g1, g2;
		while (CheckForConflicts(time, g1, g2))
		{
			if (CheckPastConflicts(conflicted_groups, g1, g2))
			{
				// if g1 and g2 conflicted before, merge them and replan without constraints
				MergeGroups(g1, g2);
				ret_val = PlanForGroups(time, g1, -1, -1);
				if (ret_val == -100)	// timeout
				{
					current_plan = AS.current_plan;
					current_plan_length = AS.current_plan_length;
					timeouted_calculations++;
					groups = groups_backup;
					break;
				}
				else if (ret_val == 0)	// ok
					continue;
			}

			conflicted_groups.push_back(make_pair(g1, g2));

			// plan for g1 while avoiding g2
			ret_val = PlanForGroups(time, g1, g2, suboptimality * ComputeGroupSOC(time, g2, g1));
			if (ret_val == -100)	// timeout
			{
				current_plan = AS.current_plan;
				current_plan_length = AS.current_plan_length;
				timeouted_calculations++;
				groups = groups_backup;
				break;
			}
			else if (ret_val == 0)	// ok
				continue;

			// plan for g2 while avoiding g1
			ret_val = PlanForGroups(time, g2, g1, suboptimality * ComputeGroupSOC(time, g2, g1));
			if (ret_val == -100)	// timeout
			{
				current_plan = AS.current_plan;
				current_plan_length = AS.current_plan_length;
				timeouted_calculations++;
				groups = groups_backup;
				break;
			}
			else if (ret_val == 0)	// ok
				continue;

			// if nothing is possible, merge g1 and g2 and replan without constraints
			MergeGroups(g1, g2);
			if (PlanForGroups(time, g1, -1, -1) == -100)
			{
				current_plan = AS.current_plan;
				current_plan_length = AS.current_plan_length;
				timeouted_calculations++;
				groups = groups_backup;
				break;
			}
		}

		ComputeNubmerOfRerouts(time, plan_backup);
		replan_times.push_back(current_runtime);
	}

	instance->PrintStatistic(statistic_file, algorithm, instance->ComputeSoC(current_plan), actually_replaned_agents, false, replan_times, timeouted_calculations);

	//instance->PrintPlan(current_plan);
	instance->CheckPlan(current_plan);

	return 0;
}


// check current plan if there are any conflict between any groups
// mark them g1 and g2 (g1 is the one with smaller index) and return true
bool IndependenceDetection::CheckForConflicts(int time, int& g1, int& g2)
{
	size_t plan_length = 0;
	for (size_t i = 0; i < current_plan.size(); i++)
		plan_length = max(plan_length, current_plan[i].size());

	if (plan_length == 0 || time > plan_length)
		return false;

	for (size_t i = time; i < plan_length; i++)
	{
		for (size_t j = 0; j < current_plan.size(); j++)
		{
			for (size_t k = j + 1; k < current_plan.size(); k++)
			{
				if (j == k)
					continue;
				if (current_plan[j][i] != -1 && current_plan[j][i] != -2 && current_plan[j][i] == current_plan[k][i])
				{
					g1 = agent_to_group[j];
					g2 = agent_to_group[k];
					if (g1 < g2)
						swap(g1, g2);
					return true;
				}
				if (i > 0 && current_plan[j][i] != -1 && current_plan[j][i] != -2 && current_plan[j][i - 1] == current_plan[k][i] && current_plan[k][i - 1] == current_plan[j][i])
				{
					g1 = agent_to_group[j];
					g2 = agent_to_group[k];
					if (g1 < g2)
						swap(g1, g2);
					return true;
				}
			}
		}
	}
	return false;
}


// check if g1 and g2 are already in conflicted groups, if so, return true (g1 is first, g2 is second)
bool IndependenceDetection::CheckPastConflicts(vector<pair<int, int> >& conflicted_groups, int g1, int g2)
{
	for (size_t i = 0; i < conflicted_groups.size(); i++)
		if (conflicted_groups[i].first == g1 && conflicted_groups[i].second == g2)
			return true;
	return false;
}


// move content of g2 to g1 and delete all of g2 (g2 must remain in the groups vector)
void IndependenceDetection::MergeGroups(int g1, int g2)
{
	groups[g1].insert(groups[g1].end(), groups[g2].begin(), groups[g2].end());

	for (size_t i = 0; i < groups[g2].size(); i++)
		agent_to_group[groups[g2][i]] = g1;

	groups[g2].erase(groups[g2].begin(), groups[g2].end());
}


// compute SOC of the current plan for two selected groups (beginning at time)
int IndependenceDetection::ComputeGroupSOC(int time, int g1, int g2)
{
	int soc = 0;
	for (size_t i = 0; i < groups[g1].size(); i++)
		for (size_t j = time; j < current_plan[groups[g1][i]].size(); j++)
			if (current_plan[groups[g1][i]][j] != -1)
				soc++;

	for (size_t i = 0; i < groups[g2].size(); i++)
		for (size_t j = time; j < current_plan[groups[g2][i]].size(); j++)
			if (current_plan[groups[g2][i]][j] != -1)
				soc++;

	return soc;
}


// check how many agents were rerouted
void IndependenceDetection::ComputeNubmerOfRerouts(int time, std::vector<std::vector<int> >& plan_backup)
{
	for (size_t i = 0; i < plan_backup.size(); i++)
	{
		for (int j = time; j < min(plan_backup[i].size(), current_plan[i].size()); j++)
		{
			if (plan_backup[i][j] != current_plan[i][j])
			{
				actually_replaned_agents++;
				break;
			}
		}
	}
}


// plan for groups g1 and g2 witl SoC or less
// if g2 == -1 -> no constraints, plan only for g1
// if SoC == -1 -> no constraint on SoC
// return -100 = timeout or error, 0 = ok, 1 = too large SoC
int IndependenceDetection::PlanForGroups(int time, int g1, int g2, int SoC)
{
	// print file
	vector<pair<int, int> > agents;
	vector<pair<int, int> > garages;
	vector<int> to_print(current_plan.size());
	int garage_id = instance->nodes;

	for (size_t i = 0; i < current_plan.size(); i++)
	{
		// is in g1 and still in graph -> print & can change path
		if (agent_to_group[i] == g1 && current_plan[i][time] != -1)
		{
			if (current_plan[i][time] >= 0)		// in transit
			{
				agents.push_back(make_pair(current_plan[i][time], instance->goal[i]));
				to_print[i] = 0;
			}
			else if (current_plan[i][time] == -2)	// in garage
			{
				agents.push_back(make_pair(garage_id, instance->goal[i]));
				garages.push_back(make_pair(garage_id, instance->start[i]));
				garage_id++;
				to_print[i] = 0;
			}
		}
		// is in g2 and still in graph -> print & can not change path
		else if (agent_to_group[i] == g2 && current_plan[i][time] != -1)
		{
			if (current_plan[i][time] >= 0)		// in transit
			{
				agents.push_back(make_pair(current_plan[i][time], instance->goal[i]));
				to_print[i] = 1;
			}
			else if (current_plan[i][time] == -2)	// in garage
			{
				agents.push_back(make_pair(garage_id, instance->goal[i]));
				garages.push_back(make_pair(garage_id, instance->start[i]));
				garage_id++;
				to_print[i] = 1;
			}
		}
		// else dont print
		else
			to_print[i] = -1;
	}
	
	instance->PrintAll(time, current_plan, garages, agents);
	instance->PrintPlan(current_plan, to_print, time);

	// run CBS algorithm with timer
	string executable;
	executable = string("cbs.exe ").append(instance->solver_input_filename).append(string(" ")).append(to_string(instance->timeout * 1000)).append(string(" > ")).append(instance->solver_output_filename);
	chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	system(executable.c_str());
	chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	current_runtime += std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

	if (current_runtime > instance->timeout * 1000)
		return -100;

	// read output file
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
			if (g2 != -1)
				return 1;
			return -100;
		}

		getline(input, line);
		stringstream ss(line);
		int agents, timesteps;
		ss >> agents;
		ss >> timesteps;

		vector<int> agent_to_update;
		vector<vector<int> > tmp_plan(agents, vector<int> (timesteps, -1));

		for (size_t i = 0; i < to_print.size(); i++)
			if (to_print[i] != -1)
				agent_to_update.push_back(i);

		for (int i = 0; i < agents; i++)
		{
			getline(input, line);
			stringstream single_agent_plan(line);
			bool reached_goal = false;
			for (int j = 0; j < timesteps; j++)
			{
				int x_location, y_location, location;
				single_agent_plan >> x_location;
				single_agent_plan >> y_location;

				if (x_location == -1 || y_location == -1)
					tmp_plan[i][j] = -1;
				else if (x_location < 0 || y_location < 0)
					tmp_plan[i][j] = -2;
				else
				{
					location = instance->int_graph[x_location][y_location];
					if (location == instance->goal[agent_to_update[i]])
						reached_goal = true;
					tmp_plan[i][j] = location;
				}

				if (reached_goal)
					break;
			}
		}
		input.close();
		string executable = "del /f ";
		executable.append(instance->solver_output_filename);
		system(executable.c_str());

		// check SoC
		if (SoC != -1 && instance->ComputeSoC(tmp_plan) > SoC)
			return 1;

		// copy and fix plan so that it is the same lenght
		int makespan = max(current_plan_length, time + timesteps);

		for (size_t i = 0; i < current_plan.size(); i++)
			current_plan[i].resize(makespan, -1);

		for (size_t i = 0; i < tmp_plan.size(); i++)
		{
			for (size_t j = 0; j < tmp_plan[i].size(); j++)
				current_plan[agent_to_update[i]][time + j] = tmp_plan[i][j];
			for (size_t j = time + timesteps; j < current_plan_length; j++)
				current_plan[agent_to_update[i]][j] = -1;
		}

		int max_makespan = 0;

		for (size_t i = 0; i < current_plan.size(); i++)
			max_makespan = max((int)current_plan[i].size(), max_makespan);

		for (size_t i = 0; i < current_plan.size(); i++)
			current_plan[i].resize(max_makespan);

		current_plan_length = max_makespan;

	}
	else
		return -100;

	return 0;
}