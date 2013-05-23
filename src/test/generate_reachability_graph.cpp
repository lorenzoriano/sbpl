#include <sbpl/discrete_space_information/environment_car.h>
#include <sbpl/utils/car_simulator.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <set>
#include <queue>

#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <boost/make_shared.hpp>
#include <cassert>

int main() {

    float map_resolution = 0.1;
    int theta_bins = 32;

    ContinuousCellPtr c1 = boost::make_shared<ContinuousCell>( 1.1, -0.6, 0.202683,
                           true, map_resolution, theta_bins,
                           false);
    ContinuousCellPtr c2 = boost::make_shared<ContinuousCell>(1.1, -1.5, 5.87782,
                           true, map_resolution, theta_bins,
                           false);
    ContinuousCellPtr c3 = boost::make_shared<ContinuousCell>(1.1, -1.46, 5.87782,
                           true, map_resolution, theta_bins,
                           false);

    boost::unordered_set<ContinuousCellPtr> set;
    typedef boost::unordered_set<ContinuousCellPtr>::iterator set_iter;

    {
        std::cout<<"Inserting element "<<*c1.get()<<std::endl;
        std::pair<set_iter, bool> res = set.insert(c1);
        if (res.second) {
            std::cout<<"Insert was successfull"<<std::endl;
        }
        else {
            std::cout<<"Elements already exist, found: "<<*res.first->get()<<std::endl;
        }
    }
    std::cout<<std::endl;

    {
        std::cout<<"Inserting element "<<*c2.get()<<std::endl;
        std::pair<set_iter, bool> res = set.insert(c2);
        if (res.second) {
            std::cout<<"Insert was successfull"<<std::endl;
        }
        else {
            std::cout<<"Elements already exist, found: "<<*res.first->get()<<std::endl;
        }
    }
    std::cout<<std::endl;

    {
        std::cout<<"Inserting element "<<*c3.get()<<std::endl;
        std::pair<set_iter, bool> res = set.insert(c3);
        if (res.second) {
            std::cout<<"Insert was successfull"<<std::endl;
        }
        else {
            std::cout<<"Elements already exist, found: "<<*res.first->get()<<std::endl;
        }
    }






}
