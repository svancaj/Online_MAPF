#include "a_star.h"

using namespace std;

typedef unordered_multimap<int, State*> MyMap;

AStar::AStar(Instance* inst)
{
	instance = inst;
}

int AStar::Solve(std::vector<std::vector<int> >& plan, std::vector<int>& agents)
{
	current_runtime = 0;
	current_plan = plan;

	if (!plan.empty())
		current_plan_length = plan[0].size();
	else
		current_plan_length = 0;

	for (size_t i = 0; i < agents.size(); i++)
	{
		long long runtime = 0;

		single_plan.clear();
		fringe = priority_queue<State*, vector<State*>, pgreater>();
		chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

		FindPath(instance->start[agents[i]], instance->goal[agents[i]], instance->appear[agents[i]]);

		chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
		runtime = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
		current_runtime += runtime;

		MergePlans(agents[i], instance->appear[agents[i]]);
	}

	//instance->PrintPlan(current_plan);
	instance->CheckPlan(current_plan);

	return 0;
}

void AStar::FindPath(int from, int to, int time)
{
	int garage = -2;
	State* first = new State();
	first->node = garage;
	first->destination = to;
	first->distance = instance->distance[from][to] + 1;
	first->time = 0;
	first->parent = NULL;

	open_nodes.insert(MyMap::value_type(first->node * first->time, first));
	fringe.push(first);

	State* current_state;

	while (!fringe.empty())
	{
		current_state = fringe.top();
		fringe.pop();

		// remove current_state from open
		auto its = open_nodes.equal_range(current_state->node * current_state->time);
		for (auto it = its.first; it != its.second; ++it)
		{
			if (current_state == it->second)
			{
				open_nodes.erase(it);
				break;
			}
		}

		closed_nodes.insert(MyMap::value_type(current_state->node * current_state->time, current_state));

		if (current_state->node == to)
		{
			ConstructPath(current_state);
			return;
		}

		GetChildren(current_state, from, time);
	}
}

void AStar::GetChildren(State* current_state, int from, int start_time)
{
	int parent_node = current_state->node;
	int parent_time = current_state->time;

	if (parent_node == -2)
		AddIfCan(current_state, from, parent_time + 1, start_time);
	else
		for (size_t i = 0; i < instance->graph[parent_node].size(); i++)
			AddIfCan(current_state, instance->graph[parent_node][i], parent_time + 1, start_time);

	AddIfCan(current_state, parent_node, parent_time + 1, start_time);
}

void AStar::AddIfCan(State* parent, int node, int time, int start_time)
{
	// is there a node colision?
	int time_in_plan = start_time + time;
	for (size_t i = 0; i < current_plan.size(); i++)
		if (node != -2 && time_in_plan < (int)current_plan[i].size() && current_plan[i][time_in_plan] == node)
			return;

	// is there a swap colision?
	for (size_t i = 0; i < current_plan.size(); i++)
		if (node != -2 && time_in_plan < (int)current_plan[i].size() && current_plan[i][time_in_plan] == parent->node && current_plan[i][time_in_plan - 1] == node)
			return;

	// is it in open?
	auto its1 = open_nodes.equal_range(node * time);
	for (auto it = its1.first; it != its1.second; ++it)
		if (node == it->second->node && time == it->second->time)
			return;

	// is it in closed?
	auto its2 = closed_nodes.equal_range(node * time);
	for (auto it = its2.first; it != its2.second; ++it)
		if (node == it->second->node && time == it->second->time)
			return;

	// add
	State* tmp = new State();
	tmp->node = node;
	tmp->destination = parent->destination;
	if (tmp->node == -2)
		tmp->distance = parent->distance;
	else
		tmp->distance = instance->distance[tmp->node][tmp->destination];
	tmp->time = time;
	tmp->parent = parent;

	open_nodes.insert(MyMap::value_type(tmp->node * tmp->time, tmp));
	fringe.push(tmp);
}

void AStar::ConstructPath(State* current_state)
{
	vector<int> path;
	while (current_state != NULL)
	{
		path.push_back(current_state->node);
		current_state = current_state->parent;
	}

	for (int i = (int)path.size() - 1; i >= 0; i--)
		single_plan.push_back(path[i]);

	// also clean all states, we do not need them any more
	for (auto it = open_nodes.begin(); it != open_nodes.end(); ++it)
		delete it->second;
	for (auto it = closed_nodes.begin(); it != closed_nodes.end(); ++it)
		delete it->second;
	open_nodes.clear();
	closed_nodes.clear();
}

void AStar::MergePlans(int agent, int time)
{
	int plan_length = max((int)(time + single_plan.size()), current_plan_length);
	current_plan.push_back(vector<int>(plan_length, -1));

	if (plan_length > current_plan_length)
		for (size_t i = 0; i < current_plan.size(); i++)
			current_plan[i].resize(plan_length, -1);

	current_plan_length = plan_length;

	for (size_t i = 0; i < single_plan.size(); i++)
		current_plan[agent][i + time] = single_plan[i];
}
