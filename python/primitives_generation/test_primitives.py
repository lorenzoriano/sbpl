import carode
import numpy as np

max_vel = 1.0
min_vel = -1.0
max_steer = np.pi/4
min_steer = -np.pi/4
car = carode.Car(0.95, max_vel, min_vel, max_steer, min_steer,
                 lin_vel=0.0, steer_angle=0)

step_time = 0.1

dest = [-1.1,    0.4,    5.445]
time_bounds = [0.01, 2.0]
ret = car.find_primitive_slsqp(dest, time_bounds)
print "Result: ", ret
control = ret[0]
car.set_control(control[0], control[1])
traj = car.simulate(control[2])[-1,:] 
print "Final position:\t", traj
print "ABS Error: ", ( abs(traj[0] - dest[0]),
                       abs(traj[1] - dest[1]),
                       abs(carode.diff_angle(traj[2], dest[2]))
                       )
