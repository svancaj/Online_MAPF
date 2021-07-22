#include <queue>
#include <list>
#include <unordered_map>
#include <stdio.h>
#include <sys/types.h>

#include "instance.h"

#ifndef AS_H
#define AS_H

class State
{
public:
	int time;
	int distance;
	int node;
	int destination;
	State* parent;
};

struct pgreater : public std::binary_function<State*, State*, bool>
{
	bool operator()(const State* lhs, const State* rhs) const
	{
		if (lhs->time + lhs->distance > rhs->time + rhs->distance)
			return true;
		if (lhs->time + lhs->distance < rhs->time + rhs->distance)
			return false;
		if (lhs->time < rhs->time)
			return true;
		if (lhs->time > rhs->time)
			return false;
		return false;
	}
};

class AStar
{
public:
	AStar(Instance*);

	int Solve(std::vector<std::vector<int> >&, std::vector<int>&);

	int current_plan_length;
	long long current_runtime;

	std::vector<std::vector<int> > current_plan;	//[agent][timestep] = node (-1 if not in the graph, -2 if in garage)

private:
	Instance* instance;

	/********************/
	/* A-star functions */
	/********************/

	void FindPath(int, int, int);
	void MergePlans(int, int);
	void ConstructPath(State*);
	void GetChildren(State*, int, int);
	void AddIfCan(State*, int, int, int);

	std::vector<int> single_plan;
	std::unordered_multimap<int, State*> closed_nodes;	//we are guaranteed to have optimal path
	std::unordered_multimap<int, State*> open_nodes;		//must check for better path
	std::priority_queue<State*, std::vector<State*>, pgreater> fringe;	//sorted open nodes
};

#endif /* AS_H */