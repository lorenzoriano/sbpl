#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>
#include <iostream>

int main() {

    ContinuousCell c(0.0, 0.0, 0.0, 0.0, 0.0,
                     0.25, 16, 5);

    std::size_t h = c.hash();
    std::cout<<"Cell: "<<c<<"\thash: "<<h<<std::endl;
    ContinuousCell c1(0.0, 0.0, 0.0, 0.2, 0.0,
                     0.25, 16, 5);

    std::size_t h1 = c1.hash();
    std::cout<<"Cell: "<<c1<<"\thash: "<<h1<<std::endl;

    Car car(0.2);
    car.setInitialState(0,0,0);
    car.setControl(0.1, 3.14/4.);
    Car::return_type ret = car.simulate(1.0, 0.1);
    std::cout<<"Position after movement: "<<ret[0]<<" "<<ret[1]<<" "<<ret[2]<<" "<<std::endl;

    EnvironmentCar env("/home/pezzotto/Projects/sbpl/car_primitives/world.cfg");
    env.loadPrimitives("/home/pezzotto/Projects/sbpl/car_primitives/primitives.txt");
    std::cout<<"Environment: "<<env<<std::endl;

}
