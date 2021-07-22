#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <sstream>
#include <algorithm>

#ifndef INSTACE_H
#define INSTACE_H

class Instance
{
public:
	void ReadInput(std::string);
	void ComputeDistance();
	int ComputeSoC(std::vector<std::vector<int> >&);
	bool IDToXY(int, int&, int&);
	int GarageToStart(int, std::vector<std::pair<int, int> >&);

	void PrintAll(int, std::vector<std::vector<int> >&, std::vector<std::pair<int, int> >&, std::vector<std::pair<int, int> >&);
	void PrintPlan(std::vector<std::vector<int> >&, std::vector<int>&, int);

	void PrintStatistic(std::string&, std::string&, int, int, bool, std::vector<long long>&, int);

	std::string input_filename;
	std::string solver_input_filename;
	std::string solver_output_filename;
	std::ofstream picat;

	int timeout;		//timeout in s

	int lower_bound_soc;

	int nodes;
	int new_agents;

	std::vector<int> start;
	std::vector<int> goal;
	std::vector<int> appear;

	std::vector<std::vector<int> > graph;		// list of neighbours
	std::vector<std::vector<int> > int_graph;	// input grid
	std::vector<std::vector<int> > distance;	// distance between all pairs of nodes (-1 if not reachable)
	
	/* DEBUG */
	void PrintPlan(std::vector<std::vector<int> >&);
	void CheckPlan(std::vector<std::vector<int> >&);
};

#endif /* INSTACE_H */