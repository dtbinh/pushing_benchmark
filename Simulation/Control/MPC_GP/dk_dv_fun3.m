function out1 = dk_dv_fun3(in1,in2,in3)
%DK_DV_FUN3
%    OUT1 = DK_DV_FUN3(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    23-Apr-2018 11:24:10

I2 = in2(1,:);
I3 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
out1 = exp((I3.*1.119289175116178-x_data2.*5.655708441921091e-1).*(I3.*1.979043273906789-x_data2).*(-1.0./2.0)-(I2.*9.489105834427072-x_data1.*8.570348613428813e-1).*(I2.*1.107201849357524e1-x_data1).*(1.0./2.0)).*(I3.*3.958086547813577-x_data2.*2.0).*(-2.451096671221595e-4);
