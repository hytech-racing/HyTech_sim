close all
clear
LapSolver;
FullCarLateralParams;
% Define the start, stop, and step size for the sweep
pid_n = 100;
pid_d = 0;
pid_i = 0;
pid_p = 0;
out = sim('Lateral Models/FullCarLateral.slx');
fprintf('Actual Laptime without PID: %.4fs\n', out.tout(end));
hold on;
subplot(2,1,1);
plot(out.iFrame.X_POS_INERTIAL.Data, out.iFrame.Y_POS_INERTIAL.Data)
axis equal
xlabel('Y [m]')
ylabel('X [m]')
subplot(2,1,2);
plot(out.tout, out.pid_vals.Yaw_Error__rad_s_.Data)

startValue = 0.1;
i_startValue = 62;
i_endValue = 300;
p_startValue = 16;
p_endValue = 16;

stepSize = 1;
pid_n = 100;
% Preallocate an array to store results if necessary
% results = zeros(((stopValue - startValue)/stepSize + 1)^3, 3); % Adjust based on what you're storing
% resultIndex = 1;

figure;
% Sweep through pid_p, pid_i, pid_d
for pid_d = 0:1:0
    for pid_i = i_startValue:3:i_endValue
        for pid_p = p_startValue:3:p_endValue
            out = sim('Lateral Models/FullCarLateral.slx');
            fprintf('Actual Laptime with PID: %.4fs\n', out.tout(end));
            fprintf('pid values %.2f %.2f %.2f\n', pid_p, pid_i, pid_d)
            fprintf('sum pid error in %.4f\n', sum(out.pid_vals.Yaw_Error__rad_s_.Data))
            hold on;
            subplot(2,1,1);
            plot(out.iFrame.X_POS_INERTIAL.Data, out.iFrame.Y_POS_INERTIAL.Data)
            axis equal
            xlabel('Y [m]')
            ylabel('X [m]')
            subplot(2,1,2);
            plot(out.tout, out.pid_vals.Yaw_Error__rad_s_.Data)
            
        end
    end
end



axis equal
xlabel('Y [m]')
ylabel('X [m]') 