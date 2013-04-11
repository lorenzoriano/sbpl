#include <iostream>
#include <cmath>
#include <sbpl/utils/car_simulator.h>
#include <sbpl/discrete_space_information/environment_car.h>

int main() {

    float map_resolution = 0.2;
    int theta_bins = 16;

    float x = -1.58541;
    float y = 0.281875;
    float th = -0.351909;

    ContinuousCell c(x, y, th, true, map_resolution, theta_bins);
    std::cout<<"Coordinates "<<x<<" "<<y<<" "<<th<<" ";
    std::cout<<"Map to: "<<c<<"\n";

    CarSimulator car(0.95);

    car.setInitialState(0.957434, -0.248888, -0.508649);
    car.setControl(-1.0, 0.21);
    CarSimulator::state_type ret = car.simulate(1.0, 0.01);
    std::cout<<"Real Dest: X: "<<ret[0]<<" Y: "<<ret[1]<<" Th: "<<ret[2]<<std::endl;
    ContinuousCell newcell(ret, true, map_resolution, theta_bins);
    std::cout<<"Cell: "<<newcell<<std::endl;
    std::cout<<"Angle error (deg): "<<diff_angle(ret[2], newcell.th())<<std::endl;
}
