#include "instance.h"

using namespace std;

void Instance::ReadInput(string f)
{
	input_filename = f;
	solver_input_filename = "instance.pi";
	solver_output_filename = "tmp.out";
	timeout = 3600; // timeout for one step in seconds

	// Read input
	ifstream in;
	in.open(input_filename);
	if (!in.is_open())
		return;

	char c_dump;
	string s_dump;

	// grid size
	int rows, columns;
	in >> rows >> c_dump >> columns;

	getline(in, s_dump);

	// graph
	int_graph = vector<vector<int> >(rows, vector<int>(columns,-1));
	nodes = 0;

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			in >> c_dump;
			if (c_dump == '.')
			{
				int_graph[i][j] = nodes;
				nodes++;
			}
		}
	}

	graph = vector<vector<int> >(nodes, vector<int>());

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < columns; j++)
		{
			if (int_graph[i][j] != -1)
			{
				// down
				if (i < rows - 1 && int_graph[i + 1][j] != -1)
				{
					graph[int_graph[i][j]].push_back(int_graph[i + 1][j]);
					graph[int_graph[i + 1][j]].push_back(int_graph[i][j]);
				}

				// left
				if (j > 0 && int_graph[i][j - 1] != -1)
				{
					graph[int_graph[i][j]].push_back(int_graph[i][j - 1]);
					graph[int_graph[i][j - 1]].push_back(int_graph[i][j]);
				}
			}
		}
	}

	/*// agents

	getline(in, s_dump);	//empty line
	getline(in, s_dump);	//"Agents:"

	in >> agents;

	for (int i = 0; i < agents; i++)
	{
		int a, s, g;
		in >> s >> c_dump >> g;
		a = 0;
		start.push_back(s);
		goal.push_back(g);
		appear.push_back(a);
	}*/

	// new agents

	getline(in, s_dump);	//empty line
	getline(in, s_dump);	//"New:"

	in >> new_agents;

	for (int i = 0; i < new_agents; i++)
	{
		int a, s, g;
		in >> a >> c_dump >> s >> c_dump >> g;
		start.push_back(s);
		goal.push_back(g);
		appear.push_back(a);
	}

	/*
	// betweenness

	getline(in, s_dump);	//empty line
	getline(in, s_dump);	//"Betweenness:"

	for (int i = 0; i < nodes; i++)
	{
		int tmp_betw;
		in >> tmp_betw;
		betweenness.push_back(tmp_betw);
		if (i < nodes - 1)
			in >> c_dump;
	}

	// initial plan

	if (agents != 0)
	{
		getline(in, s_dump);	//empty line
		getline(in, s_dump);	//"Plan:"

		int plan_size;

		in >> plan_size;
		getline(in, s_dump);	//empty line

		initial_plan = vector<vector<int> >(agents, vector<int>(plan_size));

		for (int i = 0; i < agents; i++)
		{
			for (int j = 0; j < plan_size; j++)
			{
				in >> initial_plan[i][j];
				if (j < plan_size - 1)
					in >> c_dump;
			}
		}
	}
	*/

	in.close();
	// end read instance

	ComputeDistance();
	
	for (int i = 0; i < new_agents; i++)
		lower_bound_soc += distance[start[i]][goal[i]];
}

void Instance::ComputeDistance()
{
	distance = vector<vector<int> >(graph.size(), vector<int>(graph.size(),graph.size()));

	for (size_t i = 0; i < graph.size(); i++)
		for (size_t j = 0; j < graph[i].size(); j++)
			distance[i][graph[i][j]] = 1;

	for (size_t i = 0; i < graph.size(); i++)
		distance[i][i] = 0;

	for (size_t k = 0; k < graph.size(); k++)
		for (size_t i = 0; i < graph.size(); i++)
			for (size_t j = 0; j < graph.size(); j++)
				if (distance[i][j] > distance[i][k] + distance[k][j])
					distance[i][j] = distance[i][k] + distance[k][j];

	for (size_t i = 0; i < graph.size(); i++)
		for (size_t j = 0; j < graph.size(); j++)
			if (distance[i][j] == (int)graph.size())
				distance[i][j] = -1;
}

int Instance::ComputeSoC(std::vector<std::vector<int> >& plan)
{
	int soc = 0;
	for (size_t i = 0; i < plan.size(); i++)
		for (size_t j = 0; j < plan[i].size(); j++)
			if (plan[i][j] != -1)
				soc++;
	
	return soc;
}

void Instance::PrintAll(int time, vector<vector<int> >& current_plan,
	vector<pair<int, int> >& garages, vector<pair<int, int> >& agents, vector<int>& to_print)
{
	picat.open(solver_input_filename);
	picat << "ins(Graph, State, As) =>" << endl;
	picat << "    Graph = [" << endl;
	for (size_t i = 0; i < graph.size(); i++)
	{
		if (i != 0)
			picat << "," << endl;
		picat << "    $neibs(" << i + 1 << ",[" << i + 1;

		for (size_t j = 0; j < graph[i].size(); j++)
			picat << "," << graph[i][j] + 1;

		picat << "])";
	}

	int max_node_id = graph.size() - 1;

	for (size_t i = 0; i < garages.size(); i++)
	{
		picat << "," << endl << "    $neibs(" << garages[i].first + 1 << ",[" << garages[i].first + 1 << "," << garages[i].second + 1 << "])";
		max_node_id = garages[i].first;
	}
	picat << endl << "    ]," << endl;

	// state
	vector<vector<int> > state;
	int max_time = 1;
	int parked_agent = 0;
	for (size_t i = 0; i < to_print.size(); i++)
	{
		if (to_print[i] == -1)	// do not print
			continue;

		// print begining of the plan
		if (current_plan[i][time] == -2)
		{
			state.push_back(vector<int>(1, garages[parked_agent].first));
			parked_agent++;
		}
		else
		{
			state.push_back(vector<int>(1, current_plan[i][time]));
		}

		if (to_print[i] == 1)	// print rest of the plan
		{
			for (size_t j = time + 1; j < current_plan[i].size(); j++)
			{
				if (current_plan[i][j] == -2)
					state[state.size() - 1].push_back(state[state.size() - 1][0]);
				else
					state[state.size() - 1].push_back(current_plan[i][j]);
				max_time = max(max_time, (int)j);
			}
		}
	}

	picat << "    State = new_array(" << max_time << "," << state.size() << "," << max_node_id + 1 << ")," << endl;

	for (size_t i = 0; i < state.size(); i++)
	{
		for (size_t j = 0; j < state[i].size(); j++)
		{
			for (int k = 0; k < max_node_id + 1; k++)
			{
				if (state[i][j] == k)
					picat << "    State[" << j + 1 << "," << i + 1 << "," << k + 1 << "] = 1," << endl;
				else
					picat << "    State[" << j + 1 << "," << i + 1 << "," << k + 1 << "] = 0," << endl;
			}
		}
	}

	picat << "    As = [";
	for (size_t i = 0; i < agents.size(); i++)
	{
		if (i != 0)
			picat << ",";
		picat << "(" << agents[i].first + 1 << "," << agents[i].second + 1 << ")";
	}

	picat << "].";

	picat.close();
}

bool Instance::IDToXY(int id, int& x, int& y)
{
	for (size_t i = 0; i < int_graph.size(); i++)
	{
		for (size_t j = 0; j < int_graph[i].size(); j++)
		{
			if (int_graph[i][j] == id)
			{
				x = i;
				y = j;
				return true;
			}
		}
	}
	return false;
}

int Instance::GarageToStart(int id, vector<pair<int, int> >& garages)
{
	for (size_t i = 0; i < garages.size(); i++)
	{
		if (garages[i].first == id)
			return garages[i].second;
	}
	return -1;
}

void Instance::PrintStatistic(string& statistic_file, string& algorithm_name, int SoC, int rerouted, bool fail, vector<long long>& replan_times, int timeouted_calculations)
{
	string delimiter = "_";
	ofstream statistic;
	statistic.open(statistic_file, ofstream::out | ofstream::app);
	if (statistic.is_open())
	{
		// file name
		statistic << input_filename << delimiter;

		// algorithm name
		statistic << algorithm_name << delimiter;

		// lowerbound SoC
		statistic << lower_bound_soc << delimiter;

		// total SoC
		statistic << SoC << delimiter;

		// rerouted agents
		statistic << rerouted << delimiter;

		// fail / success
		if (fail)
			statistic << "fail" << delimiter;
		else
			statistic << "success" << delimiter;

		// timeouted_calculations
		statistic << timeouted_calculations << delimiter;

		long long max = 0;
		long long sum = 0;
		for (size_t i = 0; i < replan_times.size(); i++)
		{
			sum += replan_times[i];
			if (max < replan_times[i])
				max = replan_times[i];
		}

		// total time
		statistic << sum << delimiter;

		// maximal time
		statistic << max;

		statistic << endl;

		statistic.close();
	}
}

/* DEBUG */

void Instance::PrintPlan(std::vector<std::vector<int> >& plan)
{
	for (size_t i = 0; i < plan.size(); i++)
	{
		cout << "agent " << i << ": ";
		for (size_t j = 0; j < plan[i].size(); j++)
		{
			if (plan[i][j] == -1)
				cout << "X";
			else if (plan[i][j] == -2)
				cout << "G";
			else
				cout << plan[i][j];
			if (j != plan[i].size() - 1)
				cout << "=>";
		}
		cout << endl;
	}
}

void Instance::CheckPlan(std::vector<std::vector<int> >& plan)
{
	size_t plan_length = 0;
	for (size_t i = 0; i < plan.size(); i++)
		plan_length = max(plan_length, plan[i].size());

	if (plan_length == 0)
	{
		cout << "Empty plan!" << endl;
		return;
	}

	for (size_t i = 0; i < plan_length; i++)
	{
		for (size_t j = 0; j < plan.size(); j++)
		{
			for (size_t k = 0; k < plan.size(); k++)
			{
				if (j == k)
					continue;
				if (plan[j][i] != -1 && plan[j][i] != -2 && plan[j][i] == plan[k][i])
					cout << "Node collision!!! Agents [" << j << "], [" << k << "] at time [" << i << "]" << endl;
				if (i > 0 && plan[j][i] != -1 && plan[j][i] != -2 && plan[j][i-1] == plan[k][i] && plan[k][i-1] == plan[j][i])
					cout << "Swap collision!!! Agents [" << j << "], [" << k << "] at times [" << i-1 << "," << i << "]" << endl;
			}
		}
	}
	
	
}
