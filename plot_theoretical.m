% quiver 

if exist('commands','var') == 0
    commands = compute_steps()
end

syms u
ri = (0.3960*cos(2.65*(u+1.4)));
rj = (-0.99*(sin(u+1.4)));

figure(); 
hold on; 
axis equal; 
fplot(ri, rj, [0, 3.2]);

points = [5, 20, 34, 50, 73];
for i=1:1:length(points)
    idx = points(i);
    location_x = double(subs(ri, u, commands(idx, 4)))
    location_y = double(subs(rj, u, commands(idx, 4)))
    vector_speed = [ commands(idx, 5), commands(idx, 6)];
    vector_speed = (vector_speed ./ norm(vector_speed)) * 0.5;

    vector_norm = [ commands(idx, 8),  commands(idx, 9)];
    vector_norm = (vector_norm ./ norm(vector_norm)) * 0.5;

    quiver(location_x, location_y, vector_speed(1), vector_speed(2), 'b');
    quiver(location_x, location_y, vector_norm(1), vector_norm(2), 'r');
end

% Plot Theoretical Speed 
figure(); 
hold on; 
theoretical_vl = plot(commands(:, 3), commands(:, 1));
theoretical_vr = plot(commands(:, 3), commands(:, 2));
legend([theoretical_vl,theoretical_vr], ["Theoretical (V_l)", "Theoretical (V_r)"]);
xlim([0, 13]) % 12.83 seconds to run BOD path
xlabel("Time (s)")
ylabel("Neato Wheel Speed (m/s)")

% Plot actual speed


%Plot Theoretical Wheel Velocities

%Plot Planned Robot Path


