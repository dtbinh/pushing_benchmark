clear
% clc
close all

object = Square()
load('learning_output.mat');

%build variables
xo = sym('xo', [3,1]);
rx = -object.a/2;
ry = sym('ry', [1,1]);
x = [xo;ry];
u = sym('u', [2,1]);
x_data = sym('x_data', [3,1]);
gp_input = [u;ry];

%Build symbolic functions

for lv1=1:3
    %build symbolic derivatives of kernel
    k_sym{lv1} = covFunc2(data.theta1{lv1}, x_data, gp_input./exp(data.lengthscales{lv1}(:)));
    dk_dx_sym{lv1} = jacobian(k_sym{lv1}, x);
    dk_du_sym{lv1} = jacobian(k_sym{lv1}, u);
    %convert to matlab function
    k_fun{lv1} = matlabFunction(k_sym{lv1}, 'Vars', {x,u,x_data});
    dk_dx_fun{lv1} = matlabFunction(dk_dx_sym{lv1}, 'Vars', {x,u,x_data});
    dk_du_fun{lv1} = matlabFunction(dk_du_sym{lv1}, 'Vars', {x,u,x_data});
end

%rotational kinematics
Cbi = Helper.C3_2d(x(3));
R = [transpose(Cbi) [0; 0];0 0 1];
R_fun = matlabFunction(R,  'Vars', {x});
dR_dtheta = diff(R, x(3));

x_star = [0;0;0;0];
u_star = [0.05;0];

%build large derivative matrices and gp function output

N = length(data.X{1});
D = 4;
g = [];
dg_dx = [];
dg_du = [];
for lv1=1:3
    %initialize
    dK_dx{lv1} = zeros(N,4);
    dK_du{lv1} = zeros(N,2);
    k_star{lv1} = zeros(N,1);
    K1_star_sym{lv1}=[]
    dK1_dx_star_sym{lv1}=[]
    dK1_du_star_sym{lv1}=[];
    tic
    for n=1:N
        n
        %build k_star column matrix with equilibrium data 
         k_star{lv1}(n) = k_fun{lv1}(x_star,u_star,data.X{lv1}(n,:)');
        %concat kernel derivatives and sub in equilibrium numbers
        dK_dx{lv1}(n,:) = dk_dx_fun{lv1}(x_star,u_star,data.X{lv1}(n,:)');
        dK_du{lv1}(n,:) = dk_du_fun{lv1}(x_star,u_star,data.X{lv1}(n,:)');
        %build large symbolic expression
%         K1_star_sym{lv1} = [K1_star_sym{lv1};subs(k_sym{lv1}, x_data, data.X{lv1}(n,:)')];
%         dK1_dx_star_sym{lv1} = [dK1_dx_star_sym{lv1};subs(dk_dx_sym{lv1}, x_data, data.X{lv1}(n,:)')];
%         dK1_du_star_sym{lv1} = [dK1_du_star_sym{lv1};subs(dk_du_sym{lv1}, x_data, data.X{lv1}(n,:)')];
%         dK1_du_star_fun{lv1} = [subs(dk_dx_fun{lv1}, x_data, data.X{lv1}(n,:)')];
%         dK1_dx_star_fun{lv1} = [subs(dk_du_fun{lv1}, x_data, data.X{lv1}(n,:)')];
    end
%     [~, K1star] = feval(data.covfunc1{lv1}{:}, data.theta1{lv1}, data.X{lv1}, [1 2 3]./exp(data.lengthscales{lv1}(:)'));
%     twist_b = [twist_b;K1star'*obj.data.alpha{lv1}];
                
    toc
    %concat gp output into a numerical column vector
    g = [g;transpose(k_star{lv1})*data.alpha{lv1}];
    %concat gp derivatives together
    dg_dx = [dg_dx;transpose(transpose(dK_dx{lv1})*data.alpha{lv1})];
    dg_du = [dg_du;transpose(transpose(dK_du{lv1})*data.alpha{lv1})];
    disp('building functions');
%     K1_star_Fun{lv1} = matlabFunction(K1_star_sym{1}, 'Vars', {x,u});
%     dK1_dx_star_Fun{lv1} = matlabFunction(dK1_dx_star_sym{1}, 'Vars', {x,u});
%     dK1_du_star_Fun{lv1} = matlabFunction(dK1_du_star_sym{1}, 'Vars', {x,u});
end

%build expression for dry=dx(4) (note: dry = vt-)

%% Need to account for missing expression
vbpi=u;
% vbbi=g();
% dry_dx = 0 - 0 - 0;
% dry_du = 1 - dgp2_u - diff(gp(3)xrbpb. u);
dry_dx = [0, 0, 0, -dg_dx(2,4)-rx*dg_dx(3,4)];
dry_du = [0-dg_du(2,1)-rx*dg_du(3,1), 1-dg_du(2,2)-rx*dg_du(3,2)];


dR_dx = [zeros(size(dR_dtheta))*g zeros(size(dR_dtheta))*g dR_dtheta*g zeros(size(dR_dtheta))*0*g];
dR_dx_fun =  matlabFunction(dR_dx, 'Vars', {x});

A = [dR_dx_fun(x_star) + R_fun(x_star)*dg_dx; dry_dx];
B = [R_fun(x_star)*dg_du; dry_du];

clear xo rx ry x u x_data gp_input Cbi


return
fb = twist_b_gp(gp_input);
dfb_dx = jacobian(fb, x);
dfb_du = jacobian(fb, u);
dfb_dx_fun = matlabFunction(dfb_dx, 'Vars', {x,u});
dfb_du_fun = matlabFunction(dfb_du, 'Vars', {x,u});

2
fi = R*fb;
dfi_dx = jacobian(fi, x);
dfi_du = jacobian(fi, u);
dfi_dx_fun = matlabFunction(dfi_dx, 'Vars', {x,u});
dfi_du_fun = matlabFunction(dfi_du, 'Vars', {x,u});
3
vbpi_test = u;
vbbi_test = fb(1:2);
vbpb_test = vbpi_test-vbbi_test-Helper.cross3d(fi(3), [rx;ry]);
dry_test = vbpb_test(2);
dry_dx_test = jacobian(dry_test,x);
dry_du_test = jacobian(dry_test,u);
dry_dx_fun_test = matlabFunction(dry_dx_test, 'Vars', {x,u});
dry_du_fun_test = matlabFunction(dry_du_test, 'Vars', {x,u});
4
% dry_fun = matlabFunction(dry, 'Vars', {x,u});
5
f_non = [fi;dry_test];
A_fun = jacobian(f_non, x);
B_fun = jacobian(f_non, u);
6
A_fun = matlabFunction(A_fun, 'Vars', {x,u});
B_fun = matlabFunction(B_fun, 'Vars', {x,u});