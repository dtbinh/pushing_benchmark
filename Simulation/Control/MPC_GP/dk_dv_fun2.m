function out1 = dk_dv_fun2(in1,in2,in3)
%DK_DV_FUN2
%    OUT1 = DK_DV_FUN2(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 7.1.
%    23-Apr-2018 11:24:09

I2 = in2(1,:);
I3 = in2(2,:);
x_data1 = in3(1,:);
x_data2 = in3(2,:);
out1 = exp((I3.*2.677087312240504-x_data2).*(I3.*3.818648179620118-x_data2.*1.42641898983273).*(-1.0./2.0)-(I2.*4.225233887210946-x_data1).*(I2.*3.834293303079436-x_data1.*9.074748062314799e-1).*(1.0./2.0)).*(I3.*5.354174624481009-x_data2.*2.0).*(-9.440226958192061e-7);
