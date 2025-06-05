function plot_results_tabs(solver, cost_history, x_traj_hist, u_traj_hist, num_iterations)
    % Create a figure with tabs
    f = figure('Name','CDDP Results','NumberTitle','off','Position',[100 100 1000 600]);
    tgroup = uitabgroup('Parent',f);

    % Tab 1: State Trajectories
    tab1 = uitab('Parent', tgroup, 'Title', 'State Trajectories');
    axes1 = axes('Parent', tab1);
    hold(axes1, 'on');
    % Last trajectory
    xtraj = solver.x_trajectories;
    plot(axes1, xtraj(1,:), xtraj(2,:), 'b.-','LineWidth',1.5);
    rectangle(axes1,'Position',[1-0.5,1-0.5,1,1],'Curvature',[1 1],'FaceColor',[0 0.8 0.8],'EdgeColor','none');
    rectangle(axes1,'Position',[2-1,2-1,2,2],'Curvature',[1 1],'FaceColor',[0 0.8 0.8],'EdgeColor','none');
    plot(axes1, solver.system.goal(1), solver.system.goal(2), 'rx', 'MarkerSize',12,'LineWidth',2);
    xlabel(axes1,'x'); ylabel(axes1,'y');
    title(axes1,'State Trajectories');
    axis(axes1,'equal'); xlim(axes1,[-1,4]); ylim(axes1,[-1,4]);
    hold(axes1, 'off');

    % Tab 2: Control Inputs
    tab2 = uitab('Parent', tgroup, 'Title', 'Control Inputs');
    axes2 = axes('Parent', tab2);
    hold(axes2, 'on');
    T = 1:size(solver.u_trajectories,2);
    plot(axes2, T, solver.u_trajectories(1,:), 'r', 'LineWidth',1.5, 'DisplayName','u_1');
    plot(axes2, T, solver.u_trajectories(2,:), 'b', 'LineWidth',1.5, 'DisplayName','u_2');
    xlabel(axes2,'Time step'); ylabel(axes2,'Control');
    title(axes2,'Control Inputs');
    legend(axes2,'show');
    grid(axes2,'on');
    hold(axes2, 'off');

    % Tab 3: Cost Function
    tab3 = uitab('Parent', tgroup, 'Title', 'Cost Function');
    axes3 = axes('Parent', tab3);
    plot(axes3, 1:num_iterations, cost_history, 'k', 'LineWidth',1.5);
    xlabel(axes3,'Iteration'); ylabel(axes3,'Total Cost');
    title(axes3,'Cost Function over Iterations');
    grid(axes3,'on');
end