close all;

NEETO_IP = '192.168.16.81';

if exist('commands','var') == 0
    commands = compute_steps()
end

% Run the simulator 
[sensors,vels]=neato(NEETO_IP);
pause(5); % give the simulator a few seconds to load
disp("STARTING TO SEND COMMANDS!!!")
tic;
iter = 1;
time_step = commands(2,3) - commands(1,3)
while true
    loc = ceil((toc / time_step))
    send_command = [round(commands(loc, 1), 2), round(commands(loc, 2), 2)]
    vels.lrWheelVelocitiesInMetersPerSecond = send_command;
end
vels.lrWheelVelocitiesInMetersPerSecond=[0,0]; 

