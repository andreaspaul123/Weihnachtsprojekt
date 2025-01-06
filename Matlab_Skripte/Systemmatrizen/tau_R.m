function tau_R = tau_R(in1,in2,in3,in4,in5)
%tau_R
%    tau_R = tau_R(IN1,IN2,IN3,IN4,IN5)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    06-Jan-2025 16:56:45

alpha_dot = in2(1,:);
beta_dot = in2(2,:);
c = in5(4,:);
d1 = in3(8,:);
d2 = in3(9,:);
f_m = in5(3,:);
f_v = in5(2,:);
g = in3(7,:);
m1 = in3(3,:);
m2 = in3(4,:);
mu_c = in5(1,:);
tau_R = [d1.*g.*(m1+m2).*(alpha_dot.*f_v+mu_c.*atan(alpha_dot.*2.0e+1).*6.366197723675814e-1+f_m.*exp(-c.*abs(alpha_dot)).*sign(alpha_dot));d2.*g.*m2.*(beta_dot.*f_v+mu_c.*atan(beta_dot.*2.0e+1).*6.366197723675814e-1+f_m.*exp(-c.*abs(beta_dot)).*sign(beta_dot))];
end