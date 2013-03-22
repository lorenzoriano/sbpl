#ifndef CAR_SIMULATOR_H
#define CAR_SIMULATOR_H

#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <cmath>


class Car {
public:
    typedef boost::array<float, 3> state_type;

    Car(float length) {
        length_ = length;
    }

    void setControl(float lin_speed, float steer_speed) {
        lin_speed_ = lin_speed;
        steer_speed_ = steer_speed;
    }

    void setInitialState(float x, float y, float th) {
        x_ = x;
        y_ = y;
        th_ = th;
    }

    state_type simulate(float duration, float time_step) {
        state_type s;
        s[0] = x_;
        s[1] = y_;
        s[2] = th_;

        boost::numeric::odeint::integrate( boost::bind(boost::mem_fn(&Car::integrate),
                                                       this, _1, _2, _3),
                                          s,
                                          0,
                                          duration,
                                          time_step);
        return s;
    }

protected:
    void integrate(const state_type& x, state_type& dxdt, const double ) const{
        float theta = x[2];

        dxdt[0] = lin_speed_ * cos(theta); //x
        dxdt[1] = lin_speed_ * sin(theta); //y
        dxdt[2] = lin_speed_ * tan(steer_speed_) /  length_; //
    }


protected:
    float length_;
    float x_, y_, th_;

    float lin_speed_, steer_speed_;
};

#endif // CAR_SIMULATOR_H
