close all;

if exist('commands','var') == 0
    commands = compute_steps()
end

% Run the simulator 
[sensors,vels]=neatoSim();
pause(5); % give the simulator a few seconds to load
tic;
iter = 1;
while iter < size(commands,1)
    while ((iter < size(commands,1)) && commands(iter, 3) < toc)
        iter = iter + 1;
    end
    vels.lrWheelVelocitiesInMetersPerSecond = [commands(iter, 1), commands(iter, 2)];
end
vels.lrWheelVelocitiesInMetersPerSecond=[0,0]; 

