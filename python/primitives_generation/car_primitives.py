import yaml
import carode
import numpy as np
from scipy.cluster.vq import kmeans2
import copy
from collections import defaultdict
import hashlib
from matplotlib import pylab
import operator
import itertools

def cont2disc(x, numofbins, max_v, min_v):
    """Discretize an array x into numbins elements. The number will be
    fixed between min_v and max_v.
    """
    x[x > max_v] = max_v
    x[x< min_v] = min_v
    return np.round((x - min_v) / (max_v - min_v) * (numofbins-1))

def bin_angle(x, numofbins):
    """Bins an array of angles x in numbins elements.  Returns the indexes of
    the bins (that is, discrete values).
    """
    return cont2disc(x, numofbins, 2*np.pi, 0)
def continous_angle(x, numofbins):
    """Transforms a binned angle (as returned by bin_angle) into a continous
    angle.
    """
    return x * 2*np.pi / (numofbins-1)
def disc_angle(x, numofbins):
    """Discretizes an array of angles x into numbins elements. It applies
    bin_angle and then continous_angle.
    """
    binned = bin_angle(x, numofbins)
    disc = continous_angle(binned, numofbins)
    #special case with rounding around 2pi
    #def fix_edges(x):
        #if carode.diff_angle(x, 0) < 1e-2:
            #return 0
        #else:
            #return x

    #l = map(fix_edges, disc)
    #return np.array(l)
    return disc

class CarPrimitives:
    def __init__(self, cfgfile, show_plots = False):
        f = open(cfgfile, "r")
        cfg = yaml.load(f)
        self.cfg = cfg
        f.close()

        self.max_steering_angle = cfg["max_steering_angle"]
        self.min_steering_angle = cfg["min_steering_angle"]
        self.car = carode.Car(cfg["car_length"],
                              cfg["max_v"],
                              cfg["min_v"],
                              cfg["max_steering_angle"],
                              cfg["min_steering_angle"],
                              )
        self.min_duration = cfg['min_duration']
        self.max_duration = cfg['max_duration']
        self.map_resolution = cfg['map_resolution']
        self.v_bins = cfg['v_bins']
        self.theta_bins = cfg['theta_bins']
        self.show_plots = show_plots
        self.primitives = None
        self.failed_cells = None
        self.succeded_cells = None
        self.lattice = None
        self.continous_points = None
        self.primitives_costs = None

    def create_grid_map(self,
                        multiple_thetas = False,
                        only_endpoints = True):
        """Create a map of all the points reachable by sampling the velocities
        (linear and angular).

        Parameters:
        multiple_thetas: if True multiple starting orientations will be considered
        only_endpoints: if True only the final position of a trajectory is considered
        """

        car = self.car
        if multiple_thetas:
            all_thetas = np.linspace(0, 2*np.pi, self.theta_bins)
        else:
            all_thetas = [0]

        #make sure we add the zero angles
        #theoreticallly it shouldn't be necessary if the angles are symmetric around
        #0 and the number of bins is odd.

        all_thetas = np.union1d(all_thetas, [0])
        all_ws = np.union1d(np.linspace(car.min_steer, car.max_steer,
                                        self.theta_bins),
                            [0])
        #not necessary to add a zero here, the car has to move
        all_vs = np.linspace(car.min_vel, car.max_vel, self.v_bins)

        all_points = []
        for theta in all_thetas:
            #rotating the car
            car.th = theta
            for v in all_vs:
                for w in all_ws:
                    car.set_control(v, w)
                    if only_endpoints:
                        traj = car.simulate(self.max_duration)
                        all_points.append(traj[-1,:])
                    else:
                        traj = car.simulate(self.max_duration, 0.1)
                        all_points.extend(traj)

        all_points =  np.array(all_points)
        self.continous_points = all_points
        return all_points

    def lattice_from_points(self, points):
        """Creates a lattice of points by discretizing the set of points passed in."""
        map_resolution = self.map_resolution
        theta_bins = self.theta_bins

        disc_x = np.round(points[:, 0]/map_resolution) * map_resolution
        disc_y = np.round(points[:, 1]/map_resolution) * map_resolution
        disc_theta = disc_angle(points[:, 2], theta_bins)
        point_set = set((x, y, th) for x,y,th in zip(disc_x, disc_y, disc_theta)
                        if abs(x) > self.map_resolution/2 or
                        abs(y) > self.map_resolution/2)

        #remove origin
        #filtered_point_set = filter( lambda x: abs(x[0])>self/map_resolution/2 and
                                                   #abs(x[1])>self/map_resolution/2,
                                                   #point_set)

        points = np.array(list(point_set))
        self.lattice  = points
        return points

    def __test_cell_control(self, end_point, destination, control):
        theta_bin_size = 2.*np.pi / self.theta_bins
        return ((abs(end_point[0] - destination[0]) < self.map_resolution) and
                (abs(end_point[1] - destination[1]) < self.map_resolution) and
                (abs(carode.diff_angle(end_point[2], destination[2])) < theta_bin_size)
                ) and (self.min_steering_angle < control[1] < self.max_steering_angle)


    def create_spaced_primitives(self, vt_samples = None, angles_samples = None,
                                 min_t = 0.2):
        """Creates a set of primititves. Rather than using a lattice like in
        create_primitives, it regularly samples velocities and steering angles,
        then removing the redundant ones.
        """

        if angles_samples is None:
            angles_samples = self.theta_bins

        #make sure we add the zero angles
        #theoreticallly it shouldn't be necessary if the angles are symmetric around
        #0 and the number of bins is odd.
        all_angles = np.linspace(self.min_steering_angle,
                                 self.max_steering_angle,
                                 angles_samples)
        all_angles = np.union1d(all_angles, [0])

        if vt_samples is None:
            vt_samples = self.v_bins

        v_max = self.cfg['max_v']
        v_min = self.cfg['min_v']

        forward_samples = itertools.product(np.linspace(min_t, 1, vt_samples/2),
                                            all_angles)
        backward_samples = itertools.product(np.linspace(-1, -min_t, vt_samples/2),
                                             all_angles)

        forward_primitives =  [[v_max, theta, vt / v_max]
                               for (vt, theta) in forward_samples]
        backward_primitives =  [[v_min, theta, vt / v_min]
                                for (vt, theta) in backward_samples]

        all_primitives = np.vstack((forward_primitives, backward_primitives))
        prev_number = len(all_primitives)
        primitives, _ = self.remove_equivalent_primitives(all_primitives)
        print "%d primitives out of %d have been found to be equivalent " \
              "and removed" % (prev_number - len(primitives),
                                        prev_number)
        self.primitives = primitives

        if self.show_plots:
            self.plot_primitives(primitives, newfigure=True)
        return primitives


    def create_primitives(self):
        """Creates a set of primitives. The map is first discretized to a lattice,
        then for each cell a primitive is calculated. Depending on the
        discretization, many cells might fail to have resulting primitive.

        Returns the primitives
        """

        print_steps = 20
        successes = 0
        tot_attempts = 0
        max_time = self.max_duration
        min_time = self.min_duration
        primitives = []
        failed_destinations = []
        succeded_destinations = []

        continous_points = self.create_grid_map(multiple_thetas=False,
                                                only_endpoints=True)
        cells = self.lattice_from_points(continous_points)

        #we need a wide turn car to allow the optimizer to find controls outside
        #the steering range, so to discard these controls
        #wide_turn_car = copy.deepcopy(self.car)
        #wide_turn_car.max_steer = np.deg2rad(80)
        #wide_turn_car.min_steer = np.deg2rad(-80)
        car = self.car

        for cell in cells:
            tot_attempts += 1
            #print ("Testing cell %s" % cell)
            try:
                control, _, imode = car.find_primitive_slsqp_vt(cell,
                                                             [min_time,
                                                              max_time],
                                                             5)
            except ValueError:
                print "======================"
                print "Error while trying to reach ", cell
                print "======================"
                failed_destinations.append(cell)
                continue

            if imode != 0: #optimization didn't work, skipping this
                continue

            #checking the error now
            car.set_control(control[0], control[1])
            traj = car.simulate(control[2])[-1,:]
            if self.__test_cell_control(traj, cell, control):
                primitives.append(control)
                successes += 1
                succeded_destinations.append(cell)
            else:
                failed_destinations.append(cell)

            if tot_attempts % print_steps == 0:
                print "Step %d/%d, Successes: %d, Failures: %d" % (tot_attempts,
                                                                   len(cells),
                                                                   successes, len(failed_destinations))

        prev_number_of_primitives = len(primitives)
        primitives, _ = self.remove_equivalent_primitives(primitives)
        print "%d primitives out of %d have been found to be equivalent " \
               "and removed" % (prev_number_of_primitives - len(primitives),
                                prev_number_of_primitives)

        self.primitives = np.array(primitives)
        self.failed_cells = np.array(failed_destinations)
        self.succeded_cells = np.array(succeded_destinations)

        if self.show_plots:
            succeded_destinations = np.array(succeded_destinations)
            #plot of continous points
            pylab.figure()
            pylab.plot(continous_points[:,0], continous_points[:,1], '.')
            pylab.title("Reachable points (continous). # points: %d" % (len(continous_points)))

            #discrete points, with red failures
            pylab.figure()

            pylab.plot(succeded_destinations[:,0], succeded_destinations[:,1], '.')
            if self.failed_cells.size > 0:
                pylab.plot(self.failed_cells[:,0], self.failed_cells[:,1], 'rx')

            #now showing the trajectories
            trajectories = self.execute_primitives(self.primitives)
            for traj in trajectories:
                pylab.plot(traj[:,0], traj[:,1])

            pylab.title("Reachable points (lattice). # primitives: %d" % (len(self.primitives)))
            pylab.show()

        return self.primitives

    def execute_primitives(self, primitives, only_endpoint = False):
        """Executes each of the primitives and return a list of all the trajectories."""

        all_trajectories = []
        for p in primitives:
            self.car.set_control(p[0], p[1])
            traj = self.car.simulate(p[2], 0.01)
            if only_endpoint:
                all_trajectories.append(traj[-1,:])
            else:
                all_trajectories.append(traj)
        return all_trajectories

    def hash_cell(self, point):
        """Create an hash value form the discretized cell and return it as
        a hex string (see hashlib, hexdigest)
        """
        map_resolution = self.map_resolution
        theta_bins = self.theta_bins
        disc_x = np.round(point[0]/map_resolution) * map_resolution
        disc_y = np.round(point[1]/map_resolution) * map_resolution
        disc_theta = disc_angle(np.array([point[2]]), theta_bins)[0]

        h = hashlib.md5()
        h.update(str(disc_x))
        h.update(str(disc_y))
        h.update(str(disc_theta))
        return h.hexdigest()


    def remove_equivalent_primitives(self, primitives):
        """Remove all the primitives that end in the same cell after discretization.

        If two or more primitives end in the same discretized cell, the
        primitive with smallest steering angle is selected.
        To determine if two cells are the same the hash value of their
        discretization is used.

        Returns the filtered primitives
        """
        equivalence_set = defaultdict(list)
        destinations = self.execute_primitives(primitives,
                                              only_endpoint=True)
        for (point, primitive) in zip(destinations, primitives):
            hval = self.hash_cell(point)
            equivalence_set[hval].append(primitive)

        def get_theta(x):
            return np.fabs(x[1])
        def get_dist(x):
            return np.fabs(x[0] * x[2])

        new_primitives = [min(sorted(lp, key=get_dist),
                              key = get_theta)
                          for lp in equivalence_set.values()]

        return np.array(new_primitives), equivalence_set


    def cluster_primitives_from_angle(self, number_of_clusters,
                                      primitives=None,
                                      store_result = False,
                                      use_kmeans = True,
                                      angle_is_mean = False):
        if primitives is None:
            primitives = self.primitives

        #reducing some noise with roundoff
        primitives = np.round(primitives, 3)

        res_primitives = []
        #finding the classes
        angles = primitives[:,1]
        bins = np.linspace(self.min_steering_angle-0.01,
                           self.max_steering_angle+0.01,
                           number_of_clusters)
        if use_kmeans:
            centroids, classes = kmeans2(angles, bins, minit='matrix', iter=50)
        else:
            classes = np.digitize(angles, bins)
            centroids = bins

        #selecting candidates among classes
        for c in range(number_of_clusters+1):
            inclass = primitives[classes == c]
            if len(inclass) == 0:
                continue

            #distance traveled is speed * time
            distances = inclass[:,0] * inclass[:,2]

            #distinguish between positive and negative for backward movements
            forward_primitives = inclass[distances >= 0, :]
            backward_primitives = inclass[distances < 0, :]

            #find the minimum for positive primitives
            if len(forward_primitives):
                distances = forward_primitives[:,0] * forward_primitives[:,2]
                i = np.argmin(distances)
                newv = forward_primitives[i,0]
                if angle_is_mean:
                    newangle = np.mean(forward_primitives[:,1])
                else: #finding the min distance AND angle
                    min_dist = min(distances)
                    angles_with_min_dist = forward_primitives[distances == min_dist]
                    newangle_i = np.argmin(np.fabs(angles_with_min_dist[:,1]))
                    newangle = angles_with_min_dist[newangle_i,1]
                    #newangle = forward_primitives[i,1]


                newt = forward_primitives[i,2]
                res_primitives.append((newv, newangle, newt))
                #res_primitives.append(forward_primitives[i,:])

            #find the maximum for backward primitives
            if len(backward_primitives):
                distances = backward_primitives[:,0] * backward_primitives[:,2]
                i = np.argmin(np.fabs(distances))
                newv = backward_primitives[i,0]
                if angle_is_mean:
                    newangle = np.mean(backward_primitives[:,1])
                else:
                    min_dist = min(distances)
                    angles_with_min_dist = backward_primitives[distances == min_dist]
                    newangle_i = np.argmin(np.fabs(angles_with_min_dist[:,1]))
                    newangle = angles_with_min_dist[newangle_i,1]
                    #newangle = backward_primitives[i,1]

                newt = backward_primitives[i,2]
                res_primitives.append((newv, newangle, newt))
                #res_primitives.append(backward_primitives[i,:])

        new_primitives  = np.array(res_primitives)
        new_primitives, _ = np.array(self.remove_equivalent_primitives(new_primitives))
        if store_result:
            self.primitives = new_primitives
        else:
            print "Warning, the new primitives have not been stored"

        if self.show_plots:
            pylab.figure()
            if self.lattice is not None:
                pylab.plot(self.lattice[:,0], self.lattice[:,1], '.')

            #now showing the trajectories
            self.plot_primitives(new_primitives, newfigure=False)
            pylab.title("Reduced primitives. # primitives: %d" % (len(new_primitives)))
            pylab.show()

        return new_primitives

    def calculate_primitive_costs(self, primitives=None,
                                  number_of_classes = 3,
                                  backward_multiplier = 2):
        """Calculates the cost for each primitive. The main idea is that the
        higher the steering, the more a primitive costs. Backward movements
        are assigned a higher cost too. In practice this is obtained by clustering
        the angles in number_of_classes bins, each of them with an increasing
        cost. The cost for backward movements is then multiplied by
        backward_multiplier.
        """

        if primitives is None:
            primitives = self.primitives

        #the absolute curvature is what matters
        abs_angles = np.abs(primitives[:,1])
        #weird behavior of digitize
        bins = np.linspace(abs_angles.min()-0.01, abs_angles.max()+0.01,
                        number_of_classes+1)
        costs = np.digitize(abs_angles, bins)

        #increase by one all the costs but the ones that have no steering angle
        costs[abs_angles != 0] += 1

        #apply the backward multiplier
        costs[ primitives[:,0] < 0] *= backward_multiplier

        #now calculating the distances
        distances = np.abs(primitives[:,0] * primitives[:,2]) / self.map_resolution
        distances = np.ceil(distances)

        self.primitives_costs = costs * distances

        self.primitives_costs = np.array(self.primitives_costs, dtype=np.int)
        return self.primitives_costs

    def write_primitives(self, filename = None):
        motions = []
        for p, c in zip(self.primitives, self.primitives_costs):
            motions.append( {'v' : float(p[0]),
                                         'steer': float(p[1]),
                                         'duration': float(p[2]),
                                         'cost': int(c)
                                         }
                            )
        data = {'motions' : motions,
                'time_step' : 0.01
                }
        if filename is not None:
            f = open(filename, "w")
            return yaml.dump(data, f)
        else:
            return yaml.dump(data)

    def load_primitives(self, filename):
        prims = yaml.load(open(filename))
        motions = prims['motions']
        primitives = []
        costs = []
        for motion in motions:
            primitives.append([motion['v'],
                               motion['steer'],
                               motion['duration']
                               ]
                              )
            costs.append(motion['cost'])
        self.primitives = np.array(primitives)
        self.primitives_costs = np.array(costs, dtype=np.int)

    def plot_primitives(self, primitives= None, newfigure = True):
        if primitives is None:
            primitives = self.primitives

        if newfigure:
            pylab.figure()
        trajectories = self.execute_primitives(primitives)
        for traj in trajectories:
            pylab.plot(traj[:,0], traj[:,1])

        #setting the ticks manually
        old_x_ticks = pylab.xticks()[0]
        old_y_ticks = pylab.yticks()[0]
        new_x_ticks = np.arange(old_x_ticks.min(), old_x_ticks.max(),
                                self.map_resolution)
        new_x_ticks = np.r_[new_x_ticks, old_x_ticks.max()]
        new_y_ticks = np.arange(old_y_ticks.min(), old_y_ticks.max(),
                                self.map_resolution)
        new_y_ticks = np.r_[new_y_ticks, old_y_ticks.max()]
        pylab.xticks(new_x_ticks, rotation=45)
        pylab.yticks(new_y_ticks)

        pylab.grid(True)
        num_of_primitives = len(primitives)
        pylab.title("Number of primitives: %d" % (num_of_primitives))
        pylab.show()


    def write_world_file(self, filename = None):
        data = {}
        data["car_length"] = self.car.length
        data["max_v"] = self.car.max_vel
        data["min_v"] = self.car.min_vel
        data["max_steering_angle"] = self.car.max_steer
        data["min_steering_angle"] = self.car.min_steer
        data['min_duration'] = self.min_duration
        data['max_duration'] = self.max_duration
        data['map_resolution'] = self.map_resolution
        data['v_bins'] = self.v_bins
        data['theta_bins'] = self.theta_bins

        if filename is not None:
            f = open(filename, "w")
            return yaml.dump(data, f)
        else:
            return yaml.dump(data)

        return data

    def interpolate_waypoints(self, steps, intermediate_steps):
        """Given a series of waypoints, represented as a matrix with
        three columns (x, y, theta), interpolate between them by finding
        the best control from point to point and applying it for intermediate_steps
        times.

        Returns:
        the interpolated trajectory as a matrix
        """
        traj = []
        start = steps[0,:]
        traj.append(start)
        time_bounds = [self.min_duration, self.max_duration]
        for waypoint in steps[1:,:]:
            self.car.front_x = start[0]
            self.car.front_y = start[1]
            self.car.th = start[2]

            #(control, err, _) = self.car.find_primitive_slsqp_euler(waypoint, time_bounds)
            (control, _, _) = self.car.find_primitive_slsqp_euler_vt(waypoint, self.max_duration)
            self.car.set_control(control[0], control[1])
            dt = control[2] / intermediate_steps
            interp_steps = self.car.simulate(control[2], dt)
            traj.append(interp_steps[1:,])
            start = interp_steps[-1,:]

        return np.vstack(traj)

    def run(self, number_of_clusters, number_of_cost_classes,
            destination_filename = None,
            backward_multiplier = 2,
            min_t = 0.2):
        #self.create_primitives()
        self.create_spaced_primitives(min_t = min_t)
        self.cluster_primitives_from_angle(number_of_clusters,
                                           store_result=True,
                                           use_kmeans=True)
        self.calculate_primitive_costs(number_of_classes = number_of_cost_classes,
                                       backward_multiplier = backward_multiplier)
        return self.write_primitives(destination_filename)


if __name__ == "__main__":
    #import warnings
    #warnings.filterwarnings('error')
    prims = CarPrimitives("/home/pezzotto/monpal/catkin_ws/src/sbpl_car_planner/sbpl/car_primitives/world.yaml",
                          True)
    #prims.run(20, 3,
             #"/home/pezzotto/monpal/catkin_ws/src/sbpl_car_planner/sbpl/car_primitives/primitives.yaml",
             #3)
    prims.run(9,
              3,
              "/home/pezzotto/monpal/catkin_ws/src/sbpl_car_planner/sbpl/car_primitives/primitives.yaml",
              2,
              0.3)
    pylab.show()