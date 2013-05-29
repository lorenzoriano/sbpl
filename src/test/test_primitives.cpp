#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>

#include <iostream>
#include <vector>
#include <fstream>

int main() {

    EnvironmentCar env("/home/pezzotto/tmp/sbpl/car_primitives/world.yaml", true);
    env.loadPrimitives("/home/pezzotto/tmp/sbpl/car_primitives/primitives.yaml");

    env.setStart(0, 0, 0);

    std::vector<int> next_states, costs;
    env.GetSuccs(0, &next_states, &costs);

    std::cout<<"Number of states: "<<env.numStates()<<"\n";

    std::vector<int> all_states;
    for (uint i=0; i<env.numStates(); i++)
        all_states.push_back(i);

    env.saveSolutionYAML(all_states, "/home/pezzotto/graph.yaml");
    std::cout<<"Done\n";

}
