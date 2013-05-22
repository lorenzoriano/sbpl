#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>

#include <iostream>
#include <vector>
#include <fstream>

int main() {

    EnvironmentCar env("/home/pezzotto/tmp/sbpl/car_primitives/world.yaml");
    std::cout<<"Loading primitives..\n";
    env.loadPrimitives("/home/pezzotto/tmp/sbpl/car_primitives/primitives.yaml");
    std::cout<<"Environment: "<<env<<std::endl;

    env.setStart(0, 0, 0);

    std::vector<int> next_states, costs;
    env.GetSuccs(0, &next_states, &costs);

    for (unsigned int s=0; s<next_states.size(); s++) {        
        int next_id = next_states[s];

        //testing that it doesn't end in the same state
        if (next_id == 0) {
            std::cerr<<"ERROR: Primitive "<<s<<" leads to the initial state!\n";
            continue;
        }

        //now testing the inverse primitves
        try {
            motion_primitive p = env.findPrimitive(0, next_id);
            const ContinuousCellPtr& c = env.findCell(next_id);
            std::cout<<"OK: Primitive "<<s<<": "<<p<<" leads to state ("<<c<<")\n";
        }
        catch(CarException& e){
            std::cerr<<"Error while looking for inverse primitive: "<<e.what()<<std::endl;
        }
    }
}
