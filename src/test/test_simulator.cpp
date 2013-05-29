#include <iostream>
#include <cmath>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/discrete_space_information/environment_car.h>
#include <boost/functional/hash.hpp>

int main() {


    float map_resolution = 0.1;
    int theta_bins = 32;
    bool fixed_cells = false;


    ContinuousCell in_cell( 0, 0, 0,
                           true, map_resolution, theta_bins,
                           fixed_cells);
    ContinuousCell out_cell(0.2, 0, 6.08050191017379,
                           true, map_resolution, theta_bins,
                           fixed_cells);
    double v = 1.0;
    double w = -0.5;
    double t = 0.234;


    std::cout<<"In cell: "<<in_cell<<std::endl;
    std::cout<<"In cell hash: "<<boost::hash<ContinuousCell>()(in_cell)<<std::endl;
    std::cout<<"Out cell: "<<out_cell<<std::endl;
    std::cout<<"Out cell hash: "<<out_cell.hash()<<"\n";
//    std::cout<<"Equal hashes? "<<(in_cell.hash() == out_cell.hash()?"Yes":"No")<<std::endl;
//    std::cout<<"Equal Cells? "<<(in_cell == out_cell?"Yes":"No")<<std::endl;

    CarSimulator car(0.95);
    car.setInitialState(in_cell.toCarState());
    car.setControl(v, w);
    ContinuousCell sim_dest(car.simulate(t,1.0), out_cell.is_forward(), map_resolution, theta_bins,
                            fixed_cells);

    std::cout<<"Sim cell: "<<sim_dest<<std::endl;
    std::cout<<"Sim cell hash: "<<sim_dest.hash()<<"\n";
    std::cout<<"Equal hashes? "<<(sim_dest.hash() == out_cell.hash()?"Yes":"No")<<std::endl;
    std::cout<<"Equal Cells? "<<(sim_dest == out_cell?"Yes":"No")<<std::endl;

    out_cell.checkHashCollision(sim_dest);

}

