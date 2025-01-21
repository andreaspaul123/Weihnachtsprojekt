%% Überprüfen der Endposition
endpt = length(out.y.Time);
alpha_end = out.y.Data(endpt,1);
beta_end = out.y.Data(endpt,2);
y_end = [alpha_end; beta_end];
T_0EF = calc_T_0EF(y_end, l1, l2);
r_EF = T_0EF(1:3,4);
disp('Endposition:');
disp(r_EF);
r_EF_des = [(4*sqrt(6)-4*sqrt(2)-10)*(1/125);
            (-4*sqrt(6)-4*sqrt(2)-10*sqrt(3))*(1/125);
             0];
r_EF_des = double(r_EF_des);
disp('Soll-Endposition:');
disp(r_EF_des);
%
%% alpha, alpha_des plot
figure;
plot(out.y.Time, out.y.Data(:,1),'b');
hold on; 
plot(out.y_des.Time, out.y_des.Data(1,:),'r','LineStyle','--');
xlabel('Time [s]');
ylabel('Postition [rad/s]');
title('Vergleich: Alpha, Alpha_{des}');
grid on;
legend('alpha','alpha_{des}');
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 21, 10]);
%
%% beta, beta_des plot
figure;
plot(out.y.Time, out.y.Data(:,2),'b');
hold on;
plot(out.y_des.Time, out.y_des.Data(2,:),'r','LineStyle','--');
xlabel('Time [s]');
ylabel('Postition [rad/s]');
title('Vergleich: Beta, Beta_{des}');
grid on;
legend('beta','beta_{des}');
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 21, 10]);
%
%% plot tau_R, tau_R+tau_R_delta ALPHA
figure;
plot(out.tau_R.Time, out.tau_R.Data(1,:),'b');
hold on;
plot(out.wirkendesReibmoment.Time, out.wirkendesReibmoment.Data(2,:), ...
     'r','LineStyle','--');
xlabel('Time [s]');
ylabel('Moment [Nm]');
title('Vergleich Alpha: tau_R, tatsächliches Reibmoment');
grid on;
legend('tau_R','tatsächliches Reibmoment');
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 21, 10]);
%
%% plot tau_R, tau_R+tau_R_delta BETA
figure;
plot(out.tau_R.Time, out.tau_R.Data(2,:),'b');
hold on;
plot(out.wirkendesReibmoment.Time, out.wirkendesReibmoment.Data(1,:), ...
     'r','LineStyle','--');
xlabel('Time [s]');
ylabel('Moment [Nm]');
title('Vergleich Beta: tau_R, tatsächliches Reibmoment');
grid on;
legend('tau_R','tatsächliches Reibmoment');
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 21, 10]);
%


