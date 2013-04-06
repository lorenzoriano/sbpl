#ifndef CAR_SIMULATOR_H
#define CAR_SIMULATOR_H

#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <cmath>

class CarSimulator {
public:

    typedef boost::array<float, 3> state_type;

    CarSimulator(float length) {
        length_ = length;
    }

    void setControl(float lin_vel, float steer_angle) {
        lin_vel_ = lin_vel;
        steer_angle_ = steer_angle;
    }

    void setInitialState(float x, float y, float th) {
        x_ = x;
        y_ = y;
        th_ = th;
    }

    void setInitialState(const state_type& s) {
        x_ = s[0];
        y_ = s[1];
        th_ = s[2];
    }

    state_type simulate(double duration, double time_step) {
        state_type s;
        s[0] = x_;
        s[1] = y_;
        s[2] = th_;


        boost::numeric::odeint::integrate(boost::ref(*this),
                                          s,
                                          0.,
                                          duration,
                                          double(time_step));

        state_type ret;
        ret[0] = s[0];
        ret[1] = s[1];
        ret[2] = s[2];
        return ret;
    }


    void operator()(const state_type& x, state_type& dxdt, const double) {
        float theta = x[2];

        dxdt[0] = lin_vel_ * cos(theta); //x
        dxdt[1] = lin_vel_ * sin(theta); //y
        dxdt[2] = lin_vel_ * tan(steer_angle_) /  length_; //th
    }


protected:
    float length_;
    float x_, y_, th_;

    float lin_vel_, steer_angle_;

};

std::ostream& operator<<(std::ostream& stream, const CarSimulator::state_type& x) {
    for (uint i=0; i<x.size(); i++) {
        stream<<x[i]<<" ";
    }
    return stream;

}

#endif // CAR_SIMULATOR_H
