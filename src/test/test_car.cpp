#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>

#include <iostream>
#include <vector>
#include <fstream>

//timing
#include <boost/progress.hpp>

int main() {

    float end_x = 0.0;
    float end_y = 1.00;
    float end_th = 0;
    bool store_graph = false;

    EnvironmentCar env("/home/pezzotto/tmp/sbpl/car_primitives/world.yaml", store_graph);
    env.loadPrimitives("/home/pezzotto/tmp/sbpl/car_primitives/primitives.yaml");
    env.setGoal(end_x, end_y, end_th);
    env.setStart(0, 0, 0);

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

    double allocated_time_secs = 200.; // in seconds
    double initialEpsilon = 5.0;
    bool bsearchuntilfirstsolution = false;
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);

    std::vector<int> solution_stateIDs_V;

    // plan
    boost::progress_timer __timer;
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
    std::cout<<env.numStates()<<" cells expanded"<<std::endl;
    if (! bRet) {
        std::cerr<<"NO SOLUTON FOUND!\n";        
        return 1;
    }
    else
        std::cout<<"Solution found!\n";

    //priting the solution
    std::ofstream f("/home/pezzotto/cells.txt");
    for (std::vector<int>::iterator i = solution_stateIDs_V.begin(); i != solution_stateIDs_V.end(); i++) {
        const ContinuousCellPtr& c = env.findCell(*i);
        f<<*(c.get())<<" "<<c->id()<<std::endl;
    }
    f.close();

    //writing the yaml graph
    if (store_graph)
        env.saveSolutionYAML(solution_stateIDs_V, "/home/pezzotto/graph.yaml");

}
