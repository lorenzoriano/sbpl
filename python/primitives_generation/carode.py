from scipy.integrate import ode
import numpy as np
from scipy import optimize
import angles
from math import sin, cos, tan, pi, fabs
from numpy import sign

def diff_angle(a1, a2):
    return ((a1-a2) + pi) % (2*pi) - pi

def norm_angle(x):
    return angles.normalize(x, 0, 2*pi)

class Car(object):
    """A simple car simulator with the capabilities of deriving motion primitives.

    The simulation model is taken from LaValle's book, and follows the equations:
    dx = v cos(th)
    dy = v sin(th)
    dth = v tan(w) / L

    where x,y is the car's rear axle position, th is the steering angle, and
    v,w are the controlled velocity and steering angle.

    Note that the positions always refer to the front axle, and the conversion
    to the rear one is handled by the simulator.
    """
    def __init__(self, length, max_vel, min_vel, max_steer, min_steer,
                 x=0, y=0, th=0, lin_vel = 0, steer_angle=0):
        self.length = length
        self.front_x = x
        self.front_y = y
        self.th = th
        self.lin_vel = lin_vel
        self.steer_angle = steer_angle

        self.max_vel = max_vel
        self.min_vel = min_vel
        self.max_steer = max_steer
        self.min_steer = min_steer

        self.control_vel = 0
        self.control_angle = 0

    def simulate(self, duration, time_step=1.0):
        """Simulate the car for a given duration.

        The final returned trajectory will have the initial and the ending
        position, plus all the in-between poses calculated each specified
        time_step (default 1.0).

        Before using the simulate function use set_control to specify the
        applied control. The car's position is not changed by this function.
        """
        #convert to rear axle
        initial_state = [self.front_x - self.length * cos(self.th),
                         self.front_y - self.length * sin(self.th),
                         self.th]

        t = np.arange(0, duration, time_step)
        #make sure we add the final time
        if not np.allclose(t[-1], duration):
            t = np.hstack((t, duration))

        integrator = ode(self.__integrate)
        integrator.set_integrator('vode', with_jacobian=False, nsteps=1000)
        y = initial_state
        traj = [[self.front_x, self.front_y, self.th]]
        for step, prev_t in zip(t[1:], t[:-1]):
            integrator.set_initial_value(y, prev_t)
            y = integrator.integrate(step)
            if not integrator.successful():
                raise ValueError("Error during integration!")
            y[2] = norm_angle(y[2])
            #convert back to front axle
            front_axle = [y[0] + self.length * cos(self.th),
                          y[1] + self.length * sin(self.th),
                          y[2]
                          ]
            traj.append(front_axle)

        return np.array(traj)

    def set_control(self, vel, steer):
        norm_steer = angles.normalize(steer, -2*pi, 2*pi)

        self.control_vel = self.lin_vel + vel
        if self.control_vel > self.max_vel:
            self.control_vel = self.max_vel
        if self.control_vel > self.max_vel:
            self.control_vel = self.min_vel

        self.control_angle = self.steer_angle + norm_steer
        if self.control_angle > self.max_steer:
            self.control_angle = self.max_steer
        if self.control_angle > self.max_steer:
            self.control_angle = self.min_steer

    def __integrate(self,  t, state):
        theta = state[2]

        dxdt = np.empty(3)
        dxdt[0] = self.control_vel * cos(theta ) #x
        dxdt[1] = self.control_vel * sin(theta) #y
        dxdt[2] = self.control_vel * tan(self.control_angle) /  self.length #th

        dxdt[2] = dxdt[2]
        return dxdt

    def find_primitive_slsqp(self, end_pos, time_bounds,
                             num_attempts = 10):
        """Finds a primitive that leads the car to a specified position.

        The user specifies end_pos = (x,y,th) and the
        time_bounds = [min_time, max_time], i.e. how long should the control be
        executed for. A primitive is a triple of (v,w,t), i.e. velocity, steering
        angle and duration to bring the car from it's current position to the
        desired end_pos. Internally it uses scipy fmin_slsqp to find a solution.

        NOTE: time_bounds[0] should be strictly greater than 0

        Several starting conditions can be specified with the num_attempts
        parameter.

        Returns:
        a primitive (v,w,t) the brings the car from its current position to be
        as close as possible to end_pos.
        """

        #bounds
        vel_bounds = (self.min_vel + self.lin_vel, self.max_vel - self.lin_vel)
        steer_bounds = (self.min_steer + self.steer_angle, self.max_steer + self.steer_angle)

        bounds = [vel_bounds, steer_bounds, time_bounds]
        end_pos = np.array(end_pos)

        def position_error(control):
            required_time = control[2]
            if required_time <= 0:
                print "Warning, setting a time <=0"
                return np.sum(end_pos**2)

            self.set_control(control[0], control[1])

            traj = self.simulate(required_time)
            final_pos = traj[-1,:]
            err_x = (end_pos[0] - final_pos[0])
            err_y = (end_pos[1] - final_pos[1])
            err_th = diff_angle(end_pos[2], final_pos[2]) #special angles treatment

            err = err_x**2 + err_y**2 + err_th**2
            return err

        inits = zip(np.linspace(vel_bounds[0], vel_bounds[1], num_attempts),
                    [0]*num_attempts,
                    [time_bounds[0]] * num_attempts)

        best_ret = [0, np.inf, 0, 0]
        for initial_control in inits:
            ret = optimize.fmin_slsqp(position_error,
                                      initial_control,
                                      #f_eqcons=position_error,
                                      bounds=bounds,
                                      epsilon=0.0000001,
                                      iter = 1000,
                                      full_output=True,
                                      iprint=0
                                      )
            if ret[1] < best_ret[1]:
                best_ret = ret

        control, err, _, imode, _ = best_ret
        return (control, err, imode)

    def find_primitive_slsqp_euler(self, end_pos, time_bounds,
                                 num_attempts = 10):
        """Similar to find_primitive_slsqp, but the gradient is provided rather
        than approximated. It is faster than find_primitive_slsqp, but it might
        not have the correct solution as the gradient is approximated using a
        first order Euler approximation.

        See find_primitive_slsqp for the parameters and return values.
        """

        #bounds
        vel_bounds = (self.min_vel + self.lin_vel, self.max_vel - self.lin_vel)
        steer_bounds = (self.min_steer + self.steer_angle, self.max_steer + self.steer_angle)
        #time_bounds = [0.1, 1.0]

        bounds = [vel_bounds, steer_bounds, time_bounds]

        #convert end_pos to rear_axle representation
        end_pos = np.array([end_pos[0] - self.length * cos(self.th),
                            end_pos[1] - self.length * sin(self.th),
                            end_pos[2]
                            ])

        rear_x = self.front_x - self.length * cos(self.th)
        rear_y = self.front_y - self.length * sin(self.th)

        def position_error(control):
            v, w, t = control
            if t <= 0:
                print "Warning, setting a time <=0"
                return np.sum(end_pos**2)

            thp = self.th + v/self.length*t*tan(w)
            xp = rear_x + v*t*cos(thp)
            yp = rear_y + v*t*sin(thp)

            err_x = (end_pos[0] -xp)
            err_y = (end_pos[1] - yp)
            err_th = diff_angle(end_pos[2], thp) #special angles treatment

            err = err_x**2 + err_y**2 + err_th**2
            return err

        goal_x = end_pos[0]
        goal_y = end_pos[1]
        goal_th = end_pos[2]
        x0 = rear_x
        y0 = rear_y
        th0 = self.th
        l = self.length
        Abs = fabs
        def fprime(control):
            #magic coming from simpy
            v, w, t = control
            dv = -2*t*(goal_x - t*v*cos(th0) - x0)*cos(th0) - 2*t*(goal_y - t*v*sin(th0) - y0)*sin(th0) - 2*t*(-Abs(-Abs(goal_th - th0 - t*v*tan(w)/l) + pi) + pi)*tan(w)*sign(-Abs(goal_th - th0 - t*v*tan(w)/l) + pi)*sign(goal_th - th0 - t*v*tan(w)/l)/l
            dw = -2*t*v*(tan(w)**2 + 1)*(-Abs(-Abs(goal_th - th0 - t*v*tan(w)/l) + pi) + pi)*sign(-Abs(goal_th - th0 - t*v*tan(w)/l) + pi)*sign(goal_th - th0 - t*v*tan(w)/l)/l
            dt = -2*v*(goal_x - t*v*cos(th0) - x0)*cos(th0) - 2*v*(goal_y - t*v*sin(th0) - y0)*sin(th0) - 2*v*(-Abs(-Abs(goal_th - th0 - t*v*tan(w)/l) + pi) + pi)*tan(w)*sign(-Abs(goal_th - th0 - t*v*tan(w)/l) + pi)*sign(goal_th - th0 - t*v*tan(w)/l)/l

            return [dv, dw, dt]

        #initial_control = [vel_bounds[1], 0, time_bounds[0]]
        inits = zip(np.linspace(vel_bounds[0], vel_bounds[1], num_attempts),
                    [0]*num_attempts,
                    [time_bounds[0]] * num_attempts)

        best_ret = [0, np.inf, 0, 0]
        for initial_control in inits:
            ret = optimize.fmin_slsqp(position_error,
                                      initial_control,
                                      bounds=bounds,
                                      fprime=fprime,
                                      iter = 1000,
                                      full_output=True,
                                      iprint=0
                                      )
            if ret[1] < best_ret[1]:
                best_ret = ret

        control, err, _, imode, _ = best_ret
        return (control, err, imode)

if __name__ == "__main__":
    max_vel = 1.0
    min_vel = -0.3
    max_steer = np.pi/4
    min_steer = -np.pi/4
    car = Car(0.95, max_vel, min_vel, max_steer, min_steer,
                     lin_vel=0.0, steer_angle=0)
    car.set_control(0.5, 0.21)

    max_time =  1.0
    traj2 = car.simulate( max_time)
    print traj2