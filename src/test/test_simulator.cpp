#include <iostream>
#include <cmath>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/discrete_space_information/environment_car.h>
#include <boost/functional/hash.hpp>

int main() {

//    {
//        using boost::hash_combine;
//        std::size_t seed = 0;
//        hash_combine(seed, 4);
//        hash_combine(seed, 0);
//        hash_combine(seed, 30);
//        std::cout<<"First hash: "<<seed<<std::endl;

//        seed = 0;
//        hash_combine(seed, 5);
//        hash_combine(seed, 0);
//        hash_combine(seed, 30);
//        std::cout<<"Second hash: "<<seed<<std::endl;
//        return 0;

//    }

    float map_resolution = 0.1;
    int theta_bins = 32;

    CarSimulator car(0.95);
    ContinuousCell in_cell( 1.1, -0.6, 0.202683,
                           true, map_resolution, theta_bins,
                           true);
    ContinuousCell out_cell(1.1, -1.5, 5.87782,
                           true, map_resolution, theta_bins,
                           true);

    std::cout<<"In cell: "<<in_cell<<std::endl;
    std::cout<<"In cell hash: "<<in_cell.hash()<<std::endl;
    std::cout<<"Out cell: "<<out_cell<<std::endl;
    std::cout<<"Out cell hash: "<<out_cell.hash()<<"\n";


    std::cout<<"Equal hashes? "<<(in_cell.hash() == out_cell.hash()?"Yes":"No")<<std::endl;
    out_cell.checkHashCollision(in_cell);

}

