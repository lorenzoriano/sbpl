#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>

#include <iostream>
#include <vector>
#include <fstream>

int main() {

    std::cout<<"Size of int: "<<sizeof(int)<<std::endl;
    std::cout<<"Size of size_t: "<<sizeof(std::size_t)<<std::endl;

    EnvironmentCar env("/home/pezzotto/Projects/sbpl/car_primitives/world.cfg");
    env.loadPrimitives("/home/pezzotto/Projects/sbpl/car_primitives/primitives.txt");
    std::cout<<"Environment: "<<env<<std::endl;

    env.setGoal(1.0, 0, 0, 0, 0);
    env.setStart(0, 0, 0, 0, 0);

    MDPConfig mdpCfg;
    env.InitializeMDPCfg(&mdpCfg);


    SBPLPlanner* planner =  new ARAPlanner(&env, true);
    // set planner properties
    if (planner->set_start(mdpCfg.startstateid) == 0) {
        printf("ERROR: failed to set start state\n");
        throw new SBPL_Exception();
    }
    if (planner->set_goal(mdpCfg.goalstateid) == 0) {
        printf("ERROR: failed to set goal state\n");
        throw new SBPL_Exception();
    }

    double allocated_time_secs = 10.0; // in seconds
    double initialEpsilon = 3.0;
    bool bsearchuntilfirstsolution = false;
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);

    std::vector<int> solution_stateIDs_V;

    // plan
    printf("start planning...\n");
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
    printf("done planning\n");
    printf("size of solution=%d\n", (unsigned int)solution_stateIDs_V.size());

    //priting the solution
    std::ofstream f("solution.txt");
    for (std::vector<int>::iterator i = solution_stateIDs_V.begin(); i != solution_stateIDs_V.end(); i++) {
        const ContinuousCell& c = env.findCell(*i);
        f<<c.x()<<" "<<c.y()<<" "<<c.th()<<" "<<c.v()<<" "<<c.w()<<"\n";
    }
    f.close();




}
