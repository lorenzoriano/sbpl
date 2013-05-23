import carode
import sys
import numpy as np

def test_planned_cells(cells):
    max_w = 0.5236

    #put a fictionary short of pi/2 limit here
    car = carode.Car(0.95, 1.0, -0.3,
                     np.deg2rad(80), np.deg2rad(-80))
    car.front_x = cells[0, 0]
    car.front_y = cells[0, 1]
    car.th - cells[0, 2]

    for dest in cells[1:,:]:
        x, y, th = car.front_x, car.front_y, car.th
        print "Going from (%.2f, %.2f, %.2f)" % (x, y, th), " to ", dest
        (v, w, t), _, _ = car.find_primitive_slsqp(dest, [0.1,1])
        print "Control is: (%.2f, %.2f, %.2f)" % (v, w, t)
        if abs(w) > max_w:
            print "WARNING: required steering at ", w, " max is ", max_w
        car.front_x = dest[0]
        car.front_y = dest[1]
        car.th = dest[2]
        print

if __name__ == "__main__":
    filename = sys.argv[1]
    cells = np.loadtxt(filename)
    test_planned_cells(cells)

