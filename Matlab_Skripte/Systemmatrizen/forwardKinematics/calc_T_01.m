function T_01 = calc_T_01(in1,l1,l2)
%calc_T_01
%    T_01 = calc_T_01(IN1,L1,L2)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    15-Jan-2025 10:30:55

alpha = in1(1,:);
t2 = cos(alpha);
t3 = sin(alpha);
T_01 = reshape([t3,t2,0.0,0.0,-t2,t3,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0],[4,4]);
end
