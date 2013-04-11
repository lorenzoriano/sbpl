import sympy
from sympy.utilities.codegen import codegen

def diff_angle(a1, a2):
    return sympy.pi - abs(sympy.pi - abs(a1-a2))
    #return sympy.pi - (sympy.pi - (a1-a2))

x0, y0, th0 = sympy.symbols("x0, y0, th0", real=True)
v, w = sympy.symbols("v, w", real=True)
l = sympy.Symbol("l", real=True)
t = sympy.symbols("t", real=True)
goal_x,  goal_y, goal_th = sympy.symbols("goal_x,  goal_y, goal_th", real=True)

thp = th0 + v/l * t * sympy.tan(w)
xp = x0 + v*t*sympy.cos(th0)
yp = y0 + v*t*sympy.sin(th0)

err_x = goal_x  - xp
err_y = goal_y - yp
err_th = diff_angle(goal_th, thp)

err =  err_x**2 + err_y**2  + err_th**2

print "Function: "
print err

print "Derivatives (including theta error): "
print "dv: ", sympy.diff(err, v)
print "dw: ", sympy.diff(err, w)
print "dt: ", sympy.diff(err, t)



