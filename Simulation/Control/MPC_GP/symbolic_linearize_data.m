clear
%clc
close all

pusher_gp = PointPusher(.3);
object_gp = Square();
surface_gp = Surface(.35);
planar_system_gp = PlanarSystem(pusher_gp, object_gp, surface_gp);
load('learning_output_model_from_train_size_4000.mat');

%build variables
xo = sym('xo', [3,1]);
rx = -object_gp.b/2;
ry = sym('ry', [1,1]);
x = [xo;ry];
v = sym('v', [2,1]);
I = sym('I', [3,1]);
u = sym('u', [3,1]);
x_data = sym('x_data', [2,1]);
gp_input = [I(2);I(3)];
theta = sym('theta', [4,1]);

%Build symbolic functions
for lv1=1:3
    %build symbolic derivatives of kernel
    [k_sym{lv1} dk_sym{lv1}] = covFunc_data(data.theta1{lv1}, x_data, gp_input./exp(data.lengthscales{lv1}(:)), exp(data.lengthscales{lv1}(:)));
%     [k_sym{lv1} dk_sym{lv1}] = covFunc(theta, x_data, gp_input);
    dk_dphi_sym{lv1} = dk_sym{lv1}(1,:);
    dk_dc_sym{lv1} = dk_sym{lv1}(2,:);
%     dk_dx_sym_test{lv1} = jacobian(k_sym{lv1}, x);
%     dk_dv_sym_test{lv1} = jacobian(k_sym{lv1}, v);
    %convert to matlab function
    Linear.k_fun{lv1}=matlabFunction(k_sym{lv1}, 'Vars', {x,gp_input,x_data}, 'File',strcat('k_fun', num2str(lv1)));
    Linear.dk_fun{lv1}=matlabFunction(dk_sym{lv1}, 'Vars', {x,gp_input,x_data}, 'File',strcat('dk_fun', num2str(lv1)));
    Linear.dk_dphi_fun{lv1}=matlabFunction(dk_dphi_sym{lv1}, 'Vars', {x,gp_input,x_data}, 'File', strcat('dk_dx_fun', num2str(lv1)));
    Linear.dk_dc_fun{lv1}=matlabFunction(dk_dc_sym{lv1}, 'Vars', {x,gp_input,x_data}, 'File', strcat('dk_dv_fun', num2str(lv1)));
%     Linear.dk_dx_fun_test{lv1}=matlabFunction(dk_dx_sym_test{lv1}, 'Vars', {x,v,x_data}, 'File', strcat('dk_dx_fun_test', num2str(lv1)));
%     Linear.dk_dv_fun_test{lv1}=matlabFunction(dk_dv_sym_test{lv1}, 'Vars', {x,v,x_data}, 'File', strcat('dk_dv_fun_test', num2str(lv1)));
end

%rotational kinematics
Cbi = Helper.C3_2d(x(3));
Rib = [transpose(Cbi) [0; 0];0 0 1];
Rib_fun = matlabFunction(Rib,  'Vars', {x});
dR_dtheta = diff(Rib, x(3));
Linear.dRib_dtheta_fun = matlabFunction(dR_dtheta,  'Vars', {x});
%diff dxb relative to I
Ccb = Helper.C3_2d(I(2));
Rbc = [transpose(Ccb) [0; 0];0 0 1];
dRbc_dphi = diff(Rbc, I(2));
Linear.Rbc_fun = matlabFunction(Rbc,  'Vars', {gp_input});
Linear.dRbc_dphi_fun = matlabFunction(dRbc_dphi,  'Vars', {gp_input});
%convert I to v
h = [sqrt(v(1)^2+v(2)^2); atan(v(2)/v(1));object_gp.a/2-ry];
dI_dv = jacobian(h,v);
dI_dx = jacobian(h,x);
Linear.dI_dv_fun = matlabFunction(dI_dv,'Vars', {v});
Linear.dI_dx = dI_dx;
%convert v to u
v_var = planar_system_gp.Gc_fun(x)*u;
dv_dx = jacobian(v_var, x);
dv_du = jacobian(v_var, u);
Linear.dv_dx_fun = matlabFunction(dv_dx,'Vars', {x,u});
Linear.dv_du_fun = matlabFunction(dv_du,'Vars', {x});
Linear.Gc_fun = planar_system_gp.Gc_fun;

[A,B] = GP_linearization_data([0;0;0;0], [.05;0], Linear, data, object_gp);

return

return
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v_new = Linear.Gc_fun(x)*u;
gp_input = [v_new;ry];

fb = twist_b_gp(gp_input);
fb_fun = matlabFunction(fb, 'Vars', {x,u});

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
vbpi_test = v_new;
vbbi_test = fb(1:2);
vbpb_test = vbpi_test-vbbi_test-Helper.cross3d(fi(3), [rx;ry]);
dry_test = vbpb_test(2);
dry_dx_test = jacobian(dry_test,x);
dry_du_test = jacobian(dry_test,u);
dry_dx_fun_test = matlabFunction(dry_dx_test, 'Vars', {x,u});
dry_du_fun_test = matlabFunction(dry_du_test, 'Vars', {x,u});
4
% dry_fun = matlabFunction(dry, 'Vars', {x,v});
5
f_non = [fi;dry_test];
A_fun = jacobian(f_non, x);
B_fun = jacobian(f_non, u);
6
A_fun = matlabFunction(A_fun, 'Vars', {x,u});
B_fun = matlabFunction(B_fun, 'Vars', {x,u});