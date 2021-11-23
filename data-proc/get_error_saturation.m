%% map error to a saturation function
clc; clear; close all

max_vel = 0.05;

e = [0, 0.5, 1];        % normalized error
vel = [0, max_vel, 0];

% p = polyfit(e,vel,2);
p = [-4*max_vel, 4*max_vel, 0.0001];    % requires excitement on the constant term to make error converge

x = linspace(0,1,100);
plot(x, polyval(p,x));
xlim([0, 1]); ylim([0, max_vel]);
