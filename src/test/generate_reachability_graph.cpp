#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <set>
#include <queue>

int main() {
    EnvironmentCar env("/home/pezzotto/tmp/sbpl/car_primitives/world.yaml");
    env.loadPrimitives("/home/pezzotto/tmp/sbpl/car_primitives/primitives.yaml");

    env.setStart(0, 0, 0);
    float max_dim = 0.4;
    float max_x = max_dim, min_x = -max_dim;
    float max_y = max_dim, min_y = -max_dim;


    std::set<int> visited;
    std::queue<int> to_visit;

    std::ofstream nodes_file("nodes.txt");

    to_visit.push(0);
    while (! to_visit.empty()) {
        int next = to_visit.front();
        to_visit.pop();
        const ContinuousCell& start = env.findCell(next);
        nodes_file<<start.repr()<<"\n";
        std::vector<int> succs, costs;
        env.GetSuccs(next, &succs, &costs);

        for (std::vector<int>::iterator i = succs.begin(); i != succs.end(); i++) {
            int next_id = *i;
            const ContinuousCell& c = env.findCell(next_id);
            if ( (c.x()>min_x) && (c.x()<max_x) && (c.y()>min_y) && (c.y()<max_y) ) {
                std::pair<std::set<int>::iterator,bool> ret = visited.insert(next_id);
                if (ret.second) //add to elements to visit
                    to_visit.push(next_id);
            }

        }
        std::cout<<"Visited: "<<visited.size()<<" to visit: "<<to_visit.size()<<"\n";

    }
    std::cout<<"Environment size: "<<env.numStates()<<"\n";
    nodes_file.close();
}
