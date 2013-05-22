#ifndef CAR_SIMULATOR_H
#define CAR_SIMULATOR_H

#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <cmath>

/**
 * @brief An ODE to simulate a simple car model.

 * The simulation model is taken from LaValle's book, and follows the equations:
 * dx = v cos(th)
 * dy = v sin(th)
 * dth = v tan(w) / L
 *
 * where x,y is the car's rear axle position, th is the steering angle, and
 * v,w are the controlled velocity and steering angle.

 * Note that the positions always refer to the front axle, and the conversion
 * to the rear one is handled by the simulator.
 */
class CarSimulator {
public:

    typedef boost::array<float, 3> state_type;

    CarSimulator(float car_length) {
        car_length_ = car_length;
    }

    void setControl(float lin_vel, float steer_angle) {
        lin_vel_ = lin_vel;
        steer_angle_ = steer_angle;
    }

    void setInitialState(float x, float y, float th) {
        front_axle_x_ = x;
        front_axle_y_ = y;
        th_ = th;
    }

    void setInitialState(const state_type& s) {
        front_axle_x_ = s[0];
        front_axle_y_ = s[1];
        th_ = s[2];
    }

    state_type simulate(double duration, double time_step) {
        //the model tracks the position of the rear axle. Therefore we first
        //convert the pose to that point, run the model, then convert back to
        //the end pose.
        state_type rear_axle;
        rear_axle[0] = front_axle_x_ - car_length_ * cos(th_);
        rear_axle[1] = front_axle_y_ - car_length_ * sin(th_);
        rear_axle[2] = th_;


        boost::numeric::odeint::integrate(boost::ref(*this),
                                          rear_axle,
                                          0.,
                                          duration,
                                          double(time_step));

        state_type ret;
        ret[0] = rear_axle[0] + car_length_ * cos(th_);
        ret[1] = rear_axle[1] + car_length_ * sin(th_);
        ret[2] = rear_axle[2];
        return ret;
    }


    void operator()(const state_type& x, state_type& dxdt, const double) {
        float theta = x[2];

        dxdt[0] = lin_vel_ * cos(theta); //x
        dxdt[1] = lin_vel_ * sin(theta); //y
        dxdt[2] = lin_vel_ * tan(steer_angle_) /  car_length_; //th
    }

    //setter and getter functions
    float front_x() const {
        return front_axle_x_;
    }
    float front_y() const{
        return front_axle_y_;
    }
    float th() const{
        return th_;
    }
    float& front_x() {
        return front_axle_x_;
    }
    float& front_y() {
        return front_axle_y_;
    }
    float& th() {
        return th_;
    }

    void reset(state_type p) {
        front_axle_x_ = p[0];
        front_axle_y_ = p[1];
        th_ = p[2];
}


protected:
    float car_length_;
    float front_axle_x_, front_axle_y_, th_;

    float lin_vel_, steer_angle_;

};

std::ostream& operator<<(std::ostream& stream, const CarSimulator::state_type& x) {
    for (uint i=0; i<x.size(); i++) {
        stream<<x[i]<<" ";
    }
    return stream;

}

#endif // CAR_SIMULATOR_H
