function M = calc_M(in1,in2)
%calc_M
%    M = calc_M(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    22-Jan-2025 13:16:12

K1_I1 = in2(6,:);
K2_I2 = in2(7,:);
alpha = in1(1,:);
beta = in1(2,:);
l1 = in2(1,:);
l2 = in2(2,:);
m1 = in2(3,:);
m2 = in2(4,:);
t2 = cos(alpha);
t3 = cos(beta);
t4 = sin(alpha);
t5 = sin(beta);
t6 = l1.^2;
t7 = l2.^2;
t8 = l1.*t2;
t9 = l1.*t4;
t10 = t4.*t5;
t11 = t2.*t3;
t12 = t2.*t5;
t13 = t3.*t4;
t14 = -t11;
t15 = t12+t13;
t16 = t10+t14;
t17 = (l2.*t15)./2.0;
t18 = (l2.*t16)./2.0;
t19 = t8+t17;
t20 = t9+t18;
t21 = m2.*t17.*t19;
t22 = m2.*t18.*t20;
t23 = K2_I2+t21+t22;
M = reshape([K1_I1+K2_I2+m2.*t19.^2+m2.*t20.^2+(m1.*t2.^2.*t6)./4.0+(m1.*t4.^2.*t6)./4.0,t23,t23,K2_I2+(m2.*t7.*t15.^2)./4.0+(m2.*t7.*t16.^2)./4.0],[2,2]);
end
