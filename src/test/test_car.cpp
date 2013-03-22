#include <sbpl/discrete_space_information/environment_car.h>
#include <iostream>

int main() {

    EnvironmentCar env;

    ContinuousCell c(0.0, 0.0, 0.0, 0.0, 0.0,
                     0.25, 16, 5);

    std::size_t h = c.hash();
    std::cout<<"Cell: "<<c<<"\thash: "<<h<<std::endl;
    ContinuousCell c1(0.0, 0.0, 0.0, 0.2, 0.0,
                     0.25, 16, 5);

    std::size_t h1 = c1.hash();
    std::cout<<"Cell: "<<c1<<"\thash: "<<h1<<std::endl;

}
