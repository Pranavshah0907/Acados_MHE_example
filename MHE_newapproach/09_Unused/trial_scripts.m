addpath(genpath('..\..\..\acados\external\casadi-matlab'))
import casadi.*

x = MX.sym('x',2);
y = MX.sym('y');
f = Function('f',{x,y},...
           {x,sin(y)*x});
disp(f)

[r0, q0] = f(1.1,3.3);
disp(r0)
disp(q0)

g = vertcat(x,y);