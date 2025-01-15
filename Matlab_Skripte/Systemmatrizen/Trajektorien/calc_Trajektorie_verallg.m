function Trajectory = calc_Trajektorie_verallg(t)
%calc_Trajektorie_verallg
%    Trajectory = calc_Trajektorie_verallg(T)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    15-Jan-2025 11:17:32

t2 = t.^2;
t3 = t.^3;
t5 = t.^5;
t4 = t2.^2;
et1 = t.*(-3.55706166485363e-3)+t2.*5.178536647805303e-2-t3.*5.004519079781919e-2-t4.*4.221211747522119e-3+t5.*6.746024905466655e-3;
et2 = -2.565767314538405;
et3 = t.*2.034951361333008e-3+t2.*2.848551443103645e-1-t3.*7.834737281239228e-2+t4.*3.347970896281319e-2-t5.*1.605778590275211e-2;
et4 = 2.465452051359372e-1;
et5 = t.*5.625179280721946e-2+t2.*5.499674012759387e-3-t3.*2.233084633986381e-1+t4.*1.520003970872072e-1-t5.*2.435525334301928e-2;
et6 = 4.545097918511677e-4;
et7 = t.*6.141784816830919e-1-t2.*3.956761091602906e-1+t3.*3.603356918246614e-1-t4.*2.157198216856284e-1+t5.*2.880136037892808e-2;
et8 = -1.143905358343844e-3;
et9 = t.*(-9.646860549861784e-2)-t2.*2.970722918062127e-2-t3.*2.801238530171759e-2+t4.*1.25617398466985e-1-t5.*3.946044739317777e-2;
et10 = 6.727402283880776e-2;
et11 = t.*(-5.187720934933224e-1)-t2.*1.094286241359035e-1+t3.*3.085543998579637e-1-t4.*3.585629097240446e-1+t5.*9.368911779436383e-2;
et12 = 5.862411259680754e-1;
Trajectory = [et1+et2;et3+et4;et5+et6;et7+et8;et9+et10;et11+et12];
end
