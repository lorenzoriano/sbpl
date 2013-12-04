#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>

#include <iostream>
#include <vector>
#include <fstream>

int main() {

    EnvironmentCar env("/home/pezzotto/monpal/catkin_ws/src/sbpl_car_planner/sbpl/car_primitives/world.yaml", true);
    env.loadPrimitives("/home/pezzotto/monpal/catkin_ws/src/sbpl_car_planner/sbpl/car_primitives/primitives.yaml");

    env.setStart(-1.02, -0.136, 6.02);

    std::vector<int> next_states, costs;
    env.GetSuccs(0, &next_states, &costs);

    std::cout<<"Number of next states: "<<next_states.size()<<"\n";
    for (std::vector<int>::iterator i = next_states.begin(); i != next_states.end(); i++) {
        const ContinuousCellPtr& c = env.findCell(*i);
        std::cout<<*(c.get())<<" "<<c->id()<<std::endl;
    }

    std::cout<<"Done\n";

}
