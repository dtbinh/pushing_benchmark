function out1 = dk_fun1(in1,in2,in3)
%DK_FUN1
%    OUT1 = DK_FUN1(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    24-Apr-2018 10:00:27

I2 = in2(1,:);
I3 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
t2 = I2.*1.171256166694789e1;
t3 = t2-x_data1;
t4 = I2.*6.189936898045122;
t5 = t4-x_data1.*5.284870273522428e-1;
t6 = I3.*1.829553050702335;
t7 = t6-x_data2;
t8 = I3.*1.092468258237748;
t9 = t8-x_data2.*5.971230283912057e-1;
t10 = t3.*t5.*(-1.0./2.0)-t7.*t9.*(1.0./2.0);
t11 = exp(t10);
out1 = [t11.*(I2.*2.342512333389579e1-x_data1.*2.0).*(-5.061349626129383e-6);t11.*(I3.*3.659106101404671-x_data2.*2.0).*(-8.932827428557633e-7)];