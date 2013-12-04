#ifndef CAR_SIMULATOR_H
#define CAR_SIMULATOR_H

#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <cmath>

/**
 * @brief Wrap an angle between -PI and PI
 *
 * @param x the input angle
 * @return the wrapped angle
 */
template<typename scalar> inline scalar wrap_angle_negative(scalar x) {
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;

    return x - M_PI;
}

/**
 * @brief Wrap an angle between 0 and 2PI
 *
 * @param angle the input angle
 * @return the wrapped angle
 */
template<typename scalar> inline scalar wrap_angle_positive( scalar angle )
{
    scalar twoPi = 2.0 * M_PI;
    return angle - twoPi * floor( angle / twoPi );
}

template<typename scalar> inline scalar clamp(scalar x, scalar min, scalar max) {

    if (x > max)
        return max;
    else if (x < min)
        return min;
    else
        return x;
}

template<typename scalar> inline scalar clamp_wrap_positive(scalar x,
                                                   scalar min,
                                                   scalar max) {
    return clamp(wrap_angle_positive(x), min, max);
}

template<typename scalar> inline scalar clamp_wrap_negative(scalar x,
                                                   scalar min,
                                                   scalar max) {
    return clamp(wrap_angle_negative(x), min, max);
}

//Helper classes for constant values
template<typename scalar> class ConstantControl {
public:
    ConstantControl(scalar value ){
        value_ = value;
    }

    inline scalar operator()(scalar) {
        return value_;
    }

private:
    scalar value_;
};

class _nullPtr{
public:
    template<typename T>
    operator boost::shared_ptr<T>() {
        return boost::shared_ptr<T>();
    }
    template<typename T>
    T operator()(T) {
        throw(std::logic_error("nullPtr can't be called!"));
    }

};

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
template<typename scalar = double,
         typename SteeringControl = _nullPtr,
         typename VelocityControl = _nullPtr
         >
class CarSimulator {
public:

    typedef boost::array<scalar, 3> state_type;

    struct trajectory_type {
        typedef CarSimulator<scalar, SteeringControl, VelocityControl> CarSim;

        std::vector<state_type> front_states;
        std::vector<state_type> rear_states;
        std::vector<double> ts;
        scalar __car_length_;
        scalar __initial_th_;
        const CarSim* __car;


        trajectory_type(const CarSim* car, scalar car_length, scalar initial_th)
            : __car(car){
            __car_length_ = car_length;
            __initial_th_ = wrap_angle_positive(initial_th);
        }

        void operator()(const state_type& rear_axle, double t) {
            state_type front_axle;
            scalar angle = wrap_angle_positive(rear_axle[2]);
            front_axle[0] = rear_axle[0] + __car_length_ * cos(angle);
            front_axle[1] = rear_axle[1] + __car_length_ * sin(angle);
            front_axle[2] = angle;

            front_states.push_back(front_axle);
            rear_states.push_back(rear_axle);
            ts.push_back(t);
        }
    };

    CarSimulator(scalar car_length,
                 scalar steering_angle,
                 scalar velocity,
                 scalar min_steering,
                 scalar max_steering) {

        steering_control_ = _nullPtr();
        velocity_control_ = _nullPtr();
        car_length_ = car_length;
        lin_vel_ = velocity;
        steer_angle_ = steering_angle;
        min_steering_ = min_steering;
        max_steering_ = max_steering;
        setInitialState(0,0,0);
    }

    CarSimulator(scalar car_length,
                 boost::shared_ptr<SteeringControl> steering_control,
                 boost::shared_ptr<VelocityControl> velocity_control,
                 scalar min_steering,
                 scalar max_steering
                 ) {

        steering_control_ = steering_control;
        velocity_control_ = velocity_control;
        car_length_ = car_length;
        lin_vel_ = 0;
        steer_angle_ = 0;
        min_steering_ = min_steering;
        max_steering_ = max_steering;
        setInitialState(0,0,0);
    }

    void setSteeringControl(boost::shared_ptr<SteeringControl> ctrl) {
        steering_control_ = ctrl;
    }

    void setVelocityControl(boost::shared_ptr<VelocityControl> ctrl) {
        velocity_control_ = ctrl;
    }

    void setLinVel(scalar lin_vel) {
        lin_vel_ = lin_vel;
    }

    void setSteeringAngle(scalar value) {
        steer_angle_ = clamp_wrap_negative(value, min_steering_, max_steering_);
    }

    void setInitialState(scalar x, scalar y, scalar th) {
        front_axle_x_ = x;
        front_axle_y_ = y;
        th_ = wrap_angle_positive(th);
    }

    void setInitialState(const state_type& s) {
        front_axle_x_ = s[0];
        front_axle_y_ = s[1];
        th_ = wrap_angle_positive(s[2]);
    }

    state_type simulate(double duration, double time_step) const {

        if (duration <= 0) {
            state_type ret;
            ret[0] = front_axle_x_;
            ret[1] = front_axle_y_;
            ret[2] = th_;
        }

        using namespace boost::numeric::odeint;
        typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
        typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;
        controlled_stepper_type controlled_stepper;


        //the model tracks the position of the rear axle. Therefore we first
        //convert the pose to that point, run the model, then convert back to
        //the end pose.
        state_type rear_axle;
        rear_axle[0] = front_axle_x_ - car_length_ * cos(th_);
        rear_axle[1] = front_axle_y_ - car_length_ * sin(th_);
        rear_axle[2] = th_;

        integrate_adaptive(controlled_stepper,
                           boost::ref(*this),
                           rear_axle,
                           0.,
                           duration,
                           double(time_step));

        state_type ret;
        scalar angle = wrap_angle_positive(rear_axle[2]);
        ret[0] = rear_axle[0] + car_length_ * cos(angle);
        ret[1] = rear_axle[1] + car_length_ * sin(angle);
        ret[2] = angle;
        return ret;
    }

    trajectory_type simulatReturnTrajectory(double duration, double time_step) const {

        using namespace boost::numeric::odeint;        
        runge_kutta4<state_type> stepper;

        //the model tracks the position of the rear axle. Therefore we first
        //convert the pose to that point, run the model, then convert back to
        //the end pose.
        state_type rear_axle;
        rear_axle[0] = front_axle_x_ - car_length_ * cos(th_);
        rear_axle[1] = front_axle_y_ - car_length_ * sin(th_);
        rear_axle[2] = th_;

        trajectory_type trajectory(this, car_length_, th_);
        integrate_const(stepper,
                        boost::ref(*this),
                        rear_axle,
                        0.,
                        duration,
                        double(time_step),
                        boost::ref(trajectory));

        return trajectory;
    }

    scalar get_steering_angle(double t) const {
        if (steering_control_ == NULL)
            return steer_angle_;
        else
            return steering_control_->operator()(t);
    }

    scalar get_velocity(double t) const {
        if (velocity_control_ == NULL)
            return lin_vel_;
        else
            return velocity_control_->operator()(t);
    }

    void operator()(const state_type& x, state_type& dxdt, const double t) const {
        scalar theta = wrap_angle_positive(x[2]);
        scalar steer_angle = get_steering_angle(t);
        scalar lin_vel = get_velocity(t);

        steer_angle = clamp_wrap_negative(steer_angle, min_steering_, max_steering_);

        dxdt[0] = lin_vel * cos(theta); //x
        dxdt[1] = lin_vel * sin(theta); //y
        dxdt[2] = lin_vel * tan(steer_angle) /  car_length_; //th
    }

    //setter and getter functions
    scalar front_x() const {
        return front_axle_x_;
    }
    scalar front_y() const{
        return front_axle_y_;
    }
    scalar th() const{
        return th_;
    }
    void setFront_x(scalar value) {
        front_axle_x_ = value;
    }
    void setFront_y(scalar value) {
        front_axle_y_ = value;
    }
    void setTh(scalar value) {
        th_ = wrap_angle_positive(value);
    }

    void reset(state_type p) {
        front_axle_x_ = p[0];
        front_axle_y_ = p[1];
        th_ = wrap_angle_positive(p[2]);
    }


protected:
    scalar car_length_;
    scalar lin_vel_;
    scalar steer_angle_;
    scalar front_axle_x_, front_axle_y_, th_;
    scalar max_steering_, min_steering_;

    boost::shared_ptr<SteeringControl> steering_control_;
    boost::shared_ptr<VelocityControl> velocity_control_;

};

template<typename T1, typename T2, typename T3> inline std::ostream&
operator<<(std::ostream& stream, const typename CarSimulator<T1, T2, T3>::state_type& x) {
    for (uint i=0; i<x.size(); i++) {
        stream<<x[i]<<" ";
    }
    return stream;

}

#endif // CAR_SIMULATOR_H
