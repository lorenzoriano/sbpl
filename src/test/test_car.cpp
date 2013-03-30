#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/planners/araplanner.h>
#include <sbpl/utils/mdpconfig.h>

#include <iostream>
#include <vector>
#include <fstream>

int main() {

    EnvironmentCar env("/home/pezzotto/tmp/sbpl/car_primitives/world.cfg");
    env.loadPrimitives("/home/pezzotto/tmp/sbpl/car_primitives/primitives.txt");

    float end_x = 0.0;
    float end_y = 0.5;
    float end_th = 0.0;

    env.setGoal(end_x, end_y, end_th, 0, 0);
    env.setStart(0, 0, 0, 0, 0);
//    env.setStart(7.51981000e-04,   5.00436000e-01,   1.30507346e-01,
//                 0.00000000e+00,   1.49012000e-08);

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

    double allocated_time_secs = 10.; // in seconds
    double initialEpsilon = 3.0;
    bool bsearchuntilfirstsolution = false;
    planner->set_initialsolution_eps(initialEpsilon);
    planner->set_search_mode(bsearchuntilfirstsolution);

    std::vector<int> solution_stateIDs_V;

    // plan
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);
    if (! bRet) {
        std::cerr<<"NO SOLUTON FOUND!\n";
    }

    //priting the solution
    std::ofstream f("solution.txt");
    for (std::vector<int>::iterator i = solution_stateIDs_V.begin(); i != solution_stateIDs_V.end(); i++) {
        const ContinuousCell& c = env.findCell(*i);
        f<<c.x()<<" "<<c.y()<<" "<<c.th()<<" "<<c.v()<<" "<<c.steering()<<"\n";
    }
    f.close();

}
