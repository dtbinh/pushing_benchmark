function out1 = dk_dv_fun3(in1,in2,in3)
%DK_DV_FUN3
%    OUT1 = DK_DV_FUN3(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    19-Apr-2018 18:30:53

ry1 = in1(4,:);
v1 = in2(1,:);
v2 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
x_data3 = in3(3,:);
t2 = v2.*5.053266676434502;
t3 = t2-x_data2.*1.986547170951086e-1;
t4 = v2.*2.543743612196826e1;
t5 = t4-x_data2;
t6 = ry1.*4.675593182846832e1;
t7 = t6-x_data3.*4.407613732631985e-1;
t8 = ry1.*1.060799213921775e2;
t9 = t8-x_data3;
t10 = v1.*1.583667263642789e1;
t11 = t10-x_data1;
t12 = v1.*1.861349335547444;
t13 = t12-x_data1.*1.175341170635758e-1;
t14 = t3.*t5.*(-1.0./2.0)-t7.*t9.*(1.0./2.0)-t11.*t13.*(1.0./2.0);
t15 = exp(t14);
out1 = [t15.*(v1.*3.167334527285579e1-x_data1.*2.0).*(-1.158725931553301e-1);t15.*(v2.*5.087487224393653e1-x_data2.*2.0).*(-3.145756159370641e-1)];
