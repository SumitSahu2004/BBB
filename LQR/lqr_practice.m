m=1;
c=0.2;

A=[0 1;
    0 -c/m];

B=[0;
    1/m];

% ChooseQ and R
Scenario = 1;   % 1= cheap control
                % 2= expensive control
                % 3 = ignore position
switch Scenario
    case 1
        Q= diag([1 1]);
        R= [0.01];

    case 2
        Q= diag([1 1]);
        R= [1000];

    case 3 
        Q= diag([0.001 1]);
        R= [1];
    otherwise 
        error('unknown method')
end

[K,S,E] = lqr(A,B,Q,R);

% Initial Condition 
T_final = 30;
x0=[pi;
    -2];

out=sim('LQR_prac');
t= out.tout;
x1= out.sim_x(:,1);
x2= out.sim_x(:,2);
u1= out.sim_u(:,1);

%plot
figure
subplot(3,1,1)
title(['Scenario',num2str(Scenario)])
plot(t,x1,'LineWidth',2)
grid on
legend('x1')
title('Position');

subplot(3,1,2)
plot(t,x2,'LineWidth',2)
grid on
legend('x2')
title('Velocity');

subplot(3,1,3)
plot(t,u1,'LineWidth',2)
grid on
legend('u1')
title('Control');

sgtitle('Only Non-zero velocity is Expensive.', 'FontSize', 14);