#include "dirent.h"

#include "replan_single.h"
#include "replan_all.h"
#include "independence_detection.h"

using namespace std;

int main()
{
	DIR *dir1;
	struct dirent *ent1;
	if ((dir1 = opendir("../instances")) != NULL)
	{
		while ((ent1 = readdir(dir1)) != NULL)
		{
			if (string(ent1->d_name).compare(".") == 0 || string(ent1->d_name).compare("..") == 0 || string(ent1->d_name).compare("solved") == 0)
				continue;
			// Input
			Instance* inst = new Instance();
			inst->ReadInput(string("../instances/").append(string(ent1->d_name)));

			// replan single
			cout << "Replan Single solving " << string("../instances/").append(string(ent1->d_name)) << endl;
			ReplanSingle RS(inst);
			RS.Solve();

			// replan single grouped
			cout << "Replan Single Grouped solving " << string("../instances/").append(string(ent1->d_name)) << endl;
			ReplanAll RSG(inst, 1);
			RSG.Solve();

			// replan all
			cout << "Replan All solving " << string("../instances/").append(string(ent1->d_name)) << endl;
			ReplanAll RA(inst, 0);
			RA.Solve();
			
			// independence detection
			cout << "Independence Detection solving " << string("../instances/").append(string(ent1->d_name)) << endl;
			IndependenceDetection ID(inst, 1.0);
			ID.Solve();

			// suboptimal independence detection
			cout << "Suboptimal Independence Detection solving " << string("../instances/").append(string(ent1->d_name)) << endl;
			IndependenceDetection subID(inst, 1.1);
			subID.Solve();

			delete inst;
			
			string executable;
			executable = string("move ../instances/").append(string(ent1->d_name)).append(" ../instances/solved");
			system(executable.c_str());
		}
		closedir(dir1);
	}
	else
		return 1;

	/*DIR *dir2;
	struct dirent *ent2;
	if ((dir2 = opendir("../instances_bet")) != NULL)
	{
		while ((ent2 = readdir(dir2)) != NULL)
		{
			if (string(ent2->d_name).compare(".") == 0 || string(ent2->d_name).compare("..") == 0 || string(ent2->d_name).compare("solved") == 0)
				continue;
			// Input
			Instance* inst = new Instance();
			inst->ReadInput(string("../instances_bet/").append(string(ent2->d_name)));

			// replan single + betweenness
			cout << "Replan Single + betweenness solving " << string("../instances_bet/").append(string(ent2->d_name)) << endl;
			ReplanSingle RS(inst,true, false);
			RS.Solve();

			// replan all + betweenness
			cout << "Replan All + betweenness solving " << string("../instances_bet/").append(string(ent2->d_name)) << endl;
			ReplanAll RA(inst, true, false);
			RA.Solve();
			
			// independence detection + betweenness
			cout << "Independence Detection + betweenness solving " << string("../instances_bet/").append(string(ent2->d_name)) << endl;
			IndependenceDetection ID(inst, true, false);
			ID.Solve();

			// suboptimal independence detection + betweenness
			cout << "Suboptimal Independence Detection solving " << string("../instances_bet/").append(string(ent2->d_name)) << endl;
			SubIndependenceDetection subID(inst, true, false);
			subID.Solve();

			delete inst;

			string executable;
			executable = string("move ../instances_bet/").append(string(ent2->d_name)).append(" ../instances_bet/solved");
			system(executable.c_str());
		}
		closedir(dir2);
	}
	else
		return 1;*/

	return 0;
}


