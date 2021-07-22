#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>
#include <unistd.h> // opts
#include <vector>

#include "graphs.hpp"

using namespace std;

int generateInstance(int, int, int, int);
vector<vector<int> > char_to_int(vector<vector<char> >&, int&);
void fill_start_and_goal(vector<vector<char> >&, vector<vector<int> >&, vector<vector<int> >&, vector<vector<int> >&);

int main(int argc, char* argv[])
{
	char *tvalue = NULL;
	char *avalue = NULL;
	char *mvalue = NULL;
	char *xvalue = NULL;

	// parse arguments
	opterr = 0;
	char c;
	while ((c = getopt (argc, argv, "t:a:m:x:")) != -1)
	{
		switch (c)
		{
			case 't':
				tvalue = optarg;
				break;
			case 'a':
				avalue = optarg;
				break;
			case 'm':
				mvalue = optarg;
				break;
			case 'x':
				xvalue = optarg;
				break;
			case '?':
				if (optopt == 't' || optopt == 'a' || optopt == 'm' || optopt == 'x')
				{
					cerr << "Option -" << (char)optopt << " requires an argument!" << endl;
					return -1;
				}
				// unknown option - ignore it
				break;
			default:
				return -1; // should not get here;
		}
	}

	// set problem specification
	bool all_good = true;

	int type;
	int agents;
	int makespan;
	int times = 1;

	// type of map
	if (tvalue != NULL)
		type = atoi(tvalue);
	else
		all_good = false;

	// new agents
	if (avalue != NULL)
		agents = atoi(avalue);
	else
		all_good = false;

	// makespan
	if (mvalue != NULL)
		makespan = atoi(mvalue);
	else
		all_good = false;

	// times
	if (xvalue != NULL)
		times = atoi(xvalue);

	if (!all_good)
	{
		cerr << "Some argumets are missing!";

		cerr << endl;
		return -1;
	}

	srand (time(NULL));

	for (int i = 0; i < times; i++)
		generateInstance(type, agents, makespan, i);

	return 0;
}

int generateInstance(int t, int a, int m, int instance_nr)
{
	stringstream ss;
	ss << "IntersectionType" << t << "_" << a << "_" << m << "_" << instance_nr << ".in";
	string filename = ss.str();

	vector<vector<char> > intersection_map;

	switch (t)
	{
		case 1:
			intersection_map = map1;
			break;
		case 2:
			intersection_map = map2;
			break;
		case 3:
			intersection_map = map3;
			break;
		case 4:
			intersection_map = map4;
			break;
		case 5:
			intersection_map = map5;
			break;
		case 6:
			intersection_map = map6;
			break;
		case 7:
			intersection_map = map7;
			break;
		case 8:
			intersection_map = map8;
			break;
		case 9:
			intersection_map = map9;
			break;
		case 10:
			intersection_map = map10;
			break;
		case 11:
			intersection_map = map11;
			break;
		case 12:
			intersection_map = map12;
			break;
		case 13:
			intersection_map = map13;
			break;
		default:
			cerr << "Illegal type of map!" << endl;
			return -1;
	}

	int nodes;

	vector<vector<int> > graph = char_to_int(intersection_map, nodes);
	vector<vector<int> > starts(4);
	vector<vector<int> > goals(4);
	vector<vector<int> > timestep(m);
	vector<int> start;
	vector<int> goal;

	// set start and goal
	fill_start_and_goal(intersection_map, graph, starts, goals);
	for (int i = 0; i < a; i++)
	{
		int start_side = rand() % 4;
		while (starts[start_side].size() == 0)
			start_side = rand() % 4;
		start.push_back(starts[start_side][rand() % starts[start_side].size()]);

		int goal_side = rand() % 4;
		while (goal_side == start_side || goals[goal_side].size() == 0)
			goal_side = rand() % 4;
		goal.push_back(goals[goal_side][rand() % goals[goal_side].size()]);
	}	

	// set timesteps
	for (int i = 0; i < a; i++)
		timestep[rand() % m].push_back(i);

	// print
	ofstream out1(string("instances/").append(filename));
	streambuf *coutbuf1 = cout.rdbuf(); //save old buf
	cout.rdbuf(out1.rdbuf()); //redirect std::cerr to out.txt!

	cout << graph.size() << "," << graph[0].size() << endl;
	for (size_t i = 0; i < graph.size(); i++)
	{
		for (size_t j = 0; j < graph[i].size(); j++)
		{
			if (graph[i][j] == -1)
				cout << "@";
			else
				cout << ".";
		}
		cout << endl;
	}

	//cout << "Agents:" << endl;
	//cout << 0 << endl;

	cout << "New:" << endl;
	cout << a << endl;
	for (size_t i = 0; i < timestep.size(); i++)
	{
		for (size_t j = 0; j < timestep[i].size(); j++)
		{
			cout << i << "," << start[timestep[i][j]] << "," << goal[timestep[i][j]] << endl;
		}
	}

	/*cout << "Betweenness:" << endl;
	for (int i = 0; i < nodes; i++)
	{
		cout << "1";
		if (i != nodes - 1)
			cout << ",";
	}
	cout << endl;*/

	cout.rdbuf(coutbuf1); //reset to standard output again

	return 0;

}

vector<vector<int> > char_to_int(vector<vector<char> >& in_v, int& node_id)
{
	vector<vector<int> > out_v(in_v.size(), vector<int>(in_v[0].size()));
	node_id = 0;

	for (size_t i = 0; i < in_v.size(); i++)
	{
		for (size_t j = 0; j < in_v[i].size(); j++)
		{
			if (in_v[i][j] == '@')
				out_v[i][j] = -1;
			else
			{
				out_v[i][j] = node_id;
				node_id++;
			}
		}
	}

	return out_v;
}

void fill_start_and_goal(vector<vector<char> >& graph, vector<vector<int> >& int_graph, vector<vector<int> >& starts, vector<vector<int> >& goals)
{
	// up
	/*for (size_t i = 0; i < graph[0].size(); i++)
	{
		if (graph[0][i] == 's')
			starts[0].push_back(int_graph[0][i]);
		if (graph[0][i] == 'g')
			goals[0].push_back(int_graph[0][i]);
	}*/
	// right
	for (size_t i = 0; i < graph.size(); i++)
	{
		if (graph[i][graph[i].size() - 1] == 's' || graph[i][graph[i].size() - 1] == 'd')
			starts[1].push_back(int_graph[i][graph[i].size() - 1]);
		if (graph[i][graph[i].size() - 1] == 'g' || graph[i][graph[i].size() - 1] == 'd')
			goals[1].push_back(int_graph[i][graph[i].size() - 1]);
	}
	// down
	/*for (size_t i = 0; i < graph[0].size(); i++)
	{
		if (graph[graph[0].size() - 1][i] == 's')
			starts[2].push_back(int_graph[graph[0].size() - 1][i]);
		if (graph[graph[0].size() - 1][i] == 'g')
			goals[2].push_back(int_graph[graph[0].size() - 1][i]);
	}*/
	// left
	for (size_t i = 0; i < graph.size(); i++)
	{
		if (graph[i][0] == 's' || graph[i][0] == 'd')
			starts[3].push_back(int_graph[i][0]);
		if (graph[i][0] == 'g' || graph[i][0] == 'd')
			goals[3].push_back(int_graph[i][0]);
	}
}