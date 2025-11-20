% plot_results.m
% Plots angles, trajectory and control torques similar to figures in the paper.

function plot_results(simdata)
    t = simdata.time;
    X = simdata.X;
    U = simdata.U;

    figure('Name','Tilt Angle (rad)');
    plot(t, X(2,:)); xlabel('Time (s)'); ylabel('Angle (rad)');
    title('Body Tilt Angle (simulated)');

    figure('Name','Position (simulated)');
    plot(t, X(1,:)); xlabel('Time (s)'); ylabel('Position (m)');
    title('Ball Position (simulated)');

    figure('Name','Motor Torques');
    plot(t, U(1,:)); hold on; plot(t, U(2,:)); plot(t, U(3,:));
    legend('tau1','tau2','tau3'); xlabel('Time (s)'); ylabel('Torque (Nm)');
    title('Motor torques');

    % Trajectory scatter (if we had xy pos)
    if size(X,1) >= 2
        figure('Name','Trajectory'); plot(X(1,:), X(2,:)); xlabel('x'); ylabel('y'); title('Ball trajectory (x vs y)');
    end
end
