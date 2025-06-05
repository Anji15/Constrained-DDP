system = Car();
system.set_cost(zeros(4,4), 0.05*eye(2));
Q_f = eye(4);
Q_f(1,1) = 50;
Q_f(2,2) = 50;
Q_f(3,3) = 50;
Q_f(4,4) = 10;
system.set_final_cost(Q_f);

horizon = 100;
initial_state = zeros(4,1);

solver = CDDP(system, initial_state, horizon);

num_iterations = 30;
cost_history = zeros(num_iterations,1);
x_traj_hist = zeros(system.state_size, horizon+1, num_iterations);
u_traj_hist = zeros(system.control_size, horizon, num_iterations);

system.set_goal([2; 4; pi/2; 0]);
for i = 1:10
    solver.backward_pass();
    solver.forward_pass();
end

constraint2 = CircleConstraintForCar([2; 2], 1.0, system);
solver.add_constraint(constraint2);

system.set_goal([3; 3; pi/2; 0]);
for i = 1:num_iterations
    solver.backward_pass();
    solver.forward_pass();
    % Record cost, trajectory, and controls
    cost_history(i) = solver.compute_total_cost();
    x_traj_hist(:,:,i) = solver.x_trajectories;
    u_traj_hist(:,:,i) = solver.u_trajectories;
end

% PLOT ALL IN UITAB
plot_results_tabs(solver, cost_history, x_traj_hist, u_traj_hist, num_iterations);