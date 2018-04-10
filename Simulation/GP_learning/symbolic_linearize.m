clear 
clc
close all;

theta = sym('theta', [3,1]);
x_input = sym('u_input', [3,1]);
x_data = sym('u_data', [3,1]);
P = sym('P', [3,3]);

k_sym = covFunc(theta, x_data, x_input./exp(data.lengthscales{lv1}(:)));
% 
% break
% 
% u = sym('u', [2,1]);
% xo = sym('xo', [3,1]);
% ry = sym('ry', [1,1]);
% x = [xo;ry];
% 
% Cbi = Helper.C3_2d(x(3));
% R = [transpose(Cbi) [0; 0];0 0 1];
% 
% 1
% fb = twist_b_gp([u;ry]);
% 2
% fi = R*fb;
% 3
% vbpi = u;
% vbbi = fb(1:2);
% vbpb = -vbbi+vbpi;
% dry = vbpb(2);
% 4
% dry_fun = matlabFunction(dry, 'Vars', {x,u});
% 5
% f_non = [fi;dry];
% A_fun = jacobian(f_non, x);
% B_fun = jacobian(f_non, u);
% 6
% A_fun = matlabFunction(A_fun, 'Vars', {x,u});
% B_fun = matlabFunction(B_fun, 'Vars', {x,u});
% 7
% 
