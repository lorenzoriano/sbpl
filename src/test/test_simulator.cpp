#include <iostream>
#include <cmath>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/discrete_space_information/environment_car.h>

int main() {

    float map_resolution = 0.1;
    int theta_bins = 32;

    CarSimulator car(0.95);
    ContinuousCell in_cell(1.60000002384186, 0.600000023841858, 1.21610033512115,
                           false, map_resolution, 32,
                           true);
    ContinuousCell out_cell(0.1, -0.5, 1.21610033512115,
                           false, map_resolution, 32,
                           true);

    std::cout<<"In cell hash: "<<in_cell.hash()<<std::endl;
    std::cout<<"Out cell: "<<out_cell<<std::endl;
    std::cout<<"Out cell hash: "<<out_cell.hash()<<"\n";

    car.setInitialState(in_cell.x(), in_cell.y(), in_cell.th());
    car.setControl(-0.5, 0.0);
    CarSimulator::state_type ret = car.simulate(0.2, 0.01);

    std::cout<<"Simulation Dest: X: "<<ret[0]<<" Y: "<<ret[1]<<" Th: "<<ret[2]<<std::endl;
    ContinuousCell newcell(ret, false, map_resolution, theta_bins, true);
    std::cout<<"Cell: "<<newcell<<std::endl;
    std::cout<<"Newcell hash: "<<newcell.hash()<<std::endl;

    std::cout<<"Equal hashes? "<<(out_cell.hash() == newcell.hash()?"Yes":"No")<<std::endl;

}

