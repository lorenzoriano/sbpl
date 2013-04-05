#include <iostream>
#include <sbpl/utils/car_simulator.h>

int main() {

    Car car(0.95);

    car.setInitialState(0,0,0);
    car.setControl(1070.86996923, 247.26455191);
    Car::return_type ret = car.simulate(2560.00974537, 1.0);
    std::cout<<"X: "<<ret[0]<<" Y: "<<ret[1]<<" Th: "<<ret[2]<<std::endl;
}
