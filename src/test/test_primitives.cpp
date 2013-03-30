#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>

#include <iostream>
#include <vector>
#include <fstream>

int main() {

    EnvironmentCar env("/home/pezzotto/tmp/sbpl/car_primitives/world.cfg");
    std::cout<<"Loading primitives..\n";
    env.loadPrimitives("/home/pezzotto/tmp/sbpl/car_primitives/primitives.txt");
    std::cout<<"Environment: "<<env<<std::endl;

    env.setStart(0, 0, 0, 0, 0);
    const ContinuousCell& start = env.findCell(0);

    std::vector<int> next_states, costs;
    env.GetSuccs(0, &next_states, &costs);

    for (unsigned int s=0; s<next_states.size(); s++) {
        int next_id = next_states[s];
        if (next_id == 0) {
            std::cerr<<"ERROR: Primitive "<<s<<" leads to the initial state!\n";
        }
        else {
            ContinuousCell& c = env.findCell(next_id);
            std::cerr<<"OK: Primitive "<<s<<" leads to state ("<<c<<")\n";
        }
    }
}
