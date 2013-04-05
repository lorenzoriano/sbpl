from scipy.integrate import odeint, ode
import numpy as np
from scipy import optimize
import angles
from math import sin, cos, tan, pi

def diff_angle(a1, a2):
    return ((a1-a2) + pi) % (2*pi) - pi

def norm_angle(x):
    return angles.normalize(x, 0, 2*pi)

class Car(object):
    def __init__(self, length, max_vel, min_vel, max_steer, min_steer,
                 x=0, y=0, th=0, lin_vel = 0, steer_angle=0):
        self.length = length
        self.x = x
        self.y = y
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
        initial_state = [self.x, self.y, self.th]        
        
        t = np.arange(0, duration, time_step)
        #make sure we add the final time
        if not np.allclose(t[-1], duration):
            t = np.hstack((t, duration))
        
        integrator = ode(self.integrate)
        integrator.set_integrator('vode', with_jacobian=False, nsteps=1000)        
        y = initial_state 
        traj = [initial_state]
        for step, prev_t in zip(t[1:], t[:-1]):
            integrator.set_initial_value(y, prev_t)
            y = integrator.integrate(step)
            if not integrator.successful():
                raise ValueError("Error during integration!")            
            y[2] = norm_angle(y[2])
            traj.append(y)
        
        return np.array(traj)
        
        #ret = odeint(self.integrate, initial_state, t)
        #return ret
    
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
        
    def integrate(self,  t, state):
        theta = state[2]
        
        dxdt = np.empty(3)
        dxdt[0] = self.control_vel * cos(theta ) #x
        dxdt[1] = self.control_vel * sin(theta) #y
        dxdt[2] = self.control_vel * tan(self.control_angle) /  self.length #th
        
        dxdt[2] = dxdt[2]
        return dxdt
    
    def euler_step(self, state, t):
        theta = norm_angle(state[2])
        
        dxdt = np.empty(3)
        dxdt[0] = state[0] + self.control_vel * cos(theta )*t #x
        dxdt[1] = state[1] + self.control_vel * sin(theta)*t #y
        dxdt[2] = state[2] + self.control_vel * tan(self.control_angle) /  self.length * t #th
        
        dxdt[2] = norm_angle(dxdt[2])
        
        return dxdt
    
    def find_primitive_slsqp(self, end_pos, time_bounds,
                             num_attempts = 10):
        
        #bounds
        vel_bounds = (self.min_vel + self.lin_vel, self.max_vel - self.lin_vel)
        steer_bounds = (self.min_steer + self.steer_angle, self.max_steer + self.steer_angle)
        #time_bounds = [0.1, 1.0]
        
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
            #print control, " -> ", np.array([err_x, err_y, err_th])
            #return np.array([err_x, err_y, err_th])

        #initial_control = [vel_bounds[1], 0, time_bounds[0]]
        inits = zip(np.linspace(vel_bounds[0], vel_bounds[1], num_attempts),
                    [0]*num_attempts,
                    [time_bounds[0]] * num_attempts)

        best_ret = [0, np.inf, 0, 0]
        for initial_control in inits:
            ret = optimize.fmin_slsqp(position_error,
                                      initial_control,
                                      #f_eqcons=position_error,
                                      bounds=bounds,
                                      epsilon=0.000001,
                                      iter = 100,
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
    car.set_control(1070.86996923,   247.26455191)
    
    max_time =  2560.00974537
    #steps = np.linspace(1, 2560.00974537, 100)
    traj2 = car.simulate( max_time)    
    print traj2