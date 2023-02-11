close all;

NEETO_IP = '192.168.16.81';

% Run the simulator 
[sensors,vels]=neato(NEETO_IP);
pause(5); % give the simulator a few seconds to load
disp("STARTING TO SEND COMMANDS!!!")
vels.lrWheelVelocitiesInMetersPerSecond = [0.3, 0.3];

