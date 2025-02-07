function tau_R = calc_tau_R(in1,in2,in3,in4)
%calc_tau_R
%    tau_R = calc_tau_R(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    24-Jan-2025 22:41:58

F_s1 = in4(3,:);
F_s2 = in4(4,:);
alpha_dot = in2(1,:);
beta_dot = in2(2,:);
mu_v1 = in4(1,:);
mu_v2 = in4(2,:);
tau_R = [atan(alpha_dot.*2.0e+1).*(F_s1+mu_v1.*abs(alpha_dot)).*6.366197723675814e-1;atan(beta_dot.*2.0e+1).*(F_s2+mu_v2.*abs(beta_dot)).*6.366197723675814e-1];
end
