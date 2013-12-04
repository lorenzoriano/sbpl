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

//    float end_x = 2.839422;
//    float end_y = 0.925317;
//    float end_th = 0.072350;

    float start_x = 0;
    float start_y = 0;
    float start_th = 0;

    float end_x = 2.857117+0.9;
    float end_y = 2.2112;
    float end_th = -0.029857;

    bool store_graph = true;

    EnvironmentCar env("/home/pezzotto/monpal/catkin_ws/src/sbpl_car_planner/sbpl/car_primitives/world.yaml", store_graph);
    env.loadPrimitives("/home/pezzotto/monpal/catkin_ws/src/sbpl_car_planner/sbpl/car_primitives/primitives.yaml");
    env.setGoal(end_x, end_y, end_th);
    env.setStart(start_x, start_y, start_th);

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
    boost::progress_timer __timer;
    int solution_cost = 0;
    int bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V, &solution_cost);

    {
        std::cout<<"First run, number of cells: "<<env.numStates()<<"\n";
        boost::progress_timer __timer;
        bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V, &solution_cost);
        std::cout<<"Second run, number of cells: "<<env.numStates()<<"\n";
    }

    std::cout<<env.numStates()<<" cells expanded"<<std::endl;
    std::cout<<"Cost of the solution: "<<solution_cost<<std::endl;
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
