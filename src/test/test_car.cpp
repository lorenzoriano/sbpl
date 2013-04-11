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

    EnvironmentCar env("/home/pezzotto/tmp/sbpl/car_primitives/world.yaml");
    env.loadPrimitives("/home/pezzotto/tmp/sbpl/car_primitives/primitives.yaml");

    float end_x = 0.;
    float end_y = 0.0;
    float end_th = M_PI;

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

    double allocated_time_secs = 20.; // in seconds
    double initialEpsilon = 3.0;
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
    std::ofstream f("cells.txt");
    for (std::vector<int>::iterator i = solution_stateIDs_V.begin(); i != solution_stateIDs_V.end(); i++) {
        const ContinuousCell& c = env.findCell(*i);
//        f<<c.x()<<" "<<c.y()<<" "<<c.th()<<" "<<c.is_forward()<<std::endl;
        f<<c<<std::endl;
    }
    f.close();

}
