function out1 = dk_dv_fun2(in1,in2,in3)
%DK_DV_FUN2
%    OUT1 = DK_DV_FUN2(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    17-Apr-2018 12:13:35

ry1 = in1(4,:);
v1 = in2(1,:);
v2 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
x_data3 = in3(3,:);
t2 = v1.*2.767213338879429;
t3 = t2-x_data1.*4.615824127831468e-1;
t4 = v1.*5.995058005339291;
t5 = t4-x_data1;
t6 = ry1.*2.165685256372107e1;
t7 = t6-x_data3.*7.042082895246099e-1;
t8 = ry1.*3.075347576260565e1;
t9 = t8-x_data3;
t10 = v2.*1.626458377802002e1;
t11 = t10-x_data2;
t12 = v2.*5.451385378709441;
t13 = t12-x_data2.*3.351690675341136e-1;
t14 = t3.*t5.*(-1.0./2.0)-t7.*t9.*(1.0./2.0)-t11.*t13.*(1.0./2.0);
t15 = exp(t14);
out1 = [t15.*(v1.*1.199011601067858e1-x_data1.*2.0).*(-1.944396206201901e-3);t15.*(v2.*3.252916755604004e1-x_data2.*2.0).*(-3.830443030893827e-3)];
