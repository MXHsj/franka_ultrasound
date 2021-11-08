%% map joystick axis value to cartesian velocity profile
clc; clear; close all

lin_acc_max = 13.0 * 0.018;     % [m/s^2]
ang_acc_max = 25.0 * 0.018;     % [rad/s^2]

js_range = [0,1];   % normalized

lin_acc_profile = [0, ...                                        % x^0
                   lin_acc_max*(js_range(1)+js_range(end)),...   % x^1
                   -lin_acc_max];                                % x^2
               
ang_acc_profile = [0, ...                                        % x^0
                   ang_acc_max*(js_range(1)+js_range(end)),...   % x^1
                   -ang_acc_max];                                % x^2
               
lin_vel_profile = [0, ...                           % x^0
                   0, ...                           % x^1
                   (1/2)*lin_acc_profile(2), ...    % x^2
                   (1/3)*lin_acc_profile(3)];       % x^3
               
ang_vel_profile = [0, ...                           % x^0
                   0, ...                           % x^1
                   (1/2)*ang_acc_profile(2), ...    % x^2
                   (1/3)*ang_acc_profile(3)];       % x^3 

xx = 0:0.01:1;
figure('Position',[1920/2-1000/2,1080/2-400/2,1000,400])

subplot(1,2,1); 
plot(xx,lin_vel_profile(4)*xx.^3+lin_vel_profile(3)*xx.^2,'Color','#741b47','LineWidth',1.5)
% hold on
% plot(-xx,-(lin_vel_profile(4)*xx.^3+lin_vel_profile(3)*xx.^2),'Color','#741b47','LineWidth',1.5)
xlabel('joystick input'); ylabel('linear velocity [m/s]')
grid on
title('linear velocity mapping')

subplot(1,2,2);
plot(xx,ang_vel_profile(4)*xx.^3+ang_vel_profile(3)*xx.^2,'Color','#544b48','LineWidth',1.5)
% hold on
% plot(-xx,-(ang_vel_profile(4)*xx.^3+ang_vel_profile(3)*xx.^2),'Color','#544b48','LineWidth',1.5)
xlabel('joystick input'); ylabel('angular velocity [rad/s]')
grid on
title('angular velocity mapping')
