function out1 = dk_fun2(in1,in2,in3)
%DK_FUN2
%    OUT1 = DK_FUN2(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    23-Apr-2018 11:24:09

I2 = in2(1,:);
I3 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
t2 = I3.*2.677087312240504;
t3 = t2-x_data2;
t4 = I3.*3.818648179620118;
t5 = t4-x_data2.*1.42641898983273;
t6 = I2.*4.225233887210946;
t7 = t6-x_data1;
t8 = I2.*3.834293303079436;
t9 = t8-x_data1.*9.074748062314799e-1;
t10 = t3.*t5.*(-1.0./2.0)-t7.*t9.*(1.0./2.0);
t11 = exp(t10);
out1 = [t11.*(I2.*8.450467774421892-x_data1.*2.0).*(-9.478903869312893e-7);t11.*(I3.*5.354174624481009-x_data2.*2.0).*(-9.440226958192061e-7)];
