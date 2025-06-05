classdef CDDP < handle
    properties
        system
        horizon
        x_trajectories
        u_trajectories
        initial_state
        constraints
        best_J
        Q_UX
        Q_UU
        Q_U
        reg_factor
        reg_factor_u
        active_set_tol
    end
    methods
        function obj = CDDP(system, initial_state, horizon)
            obj.system = system;
            obj.horizon = horizon;
            obj.x_trajectories = zeros(system.state_size, horizon+1);
            obj.u_trajectories = zeros(system.control_size, horizon);
            obj.initial_state = initial_state(:);
            obj.constraints = {};
            obj.best_J = 10000;
            obj.Q_UX = zeros(system.control_size, system.state_size, horizon);
            obj.Q_UU = zeros(system.control_size, system.control_size, horizon);
            obj.Q_U = zeros(system.control_size, horizon);
            obj.reg_factor = 0.001;
            obj.reg_factor_u = 0.001;
            obj.active_set_tol = 0.01;
        end

        function set_initial_trajectories(obj, x_trajectories, u_trajectories)
            obj.x_trajectories = x_trajectories;
            obj.u_trajectories = u_trajectories;
        end

        function add_constraint(obj, constraint)
            obj.constraints{end+1} = constraint;
        end

        % ...
function forward_pass(obj)
    x = obj.initial_state;
    feasible = false;
    trust_region_scale = 1;
    max_trust_region_reductions = 25;
    trust_shrink_count = 0;
    while ~feasible
        feasible = true;
        current_J = 0;
        x_new_trajectories = zeros(obj.system.state_size, obj.horizon+1);
        u_new_trajectories = zeros(obj.system.control_size, obj.horizon);
        x = obj.initial_state;
        x_new_trajectories(:,1) = x;
        broke_early = false;
        for i = 1:obj.horizon
            delta_x = x - obj.x_trajectories(:,i);
            Q_ux = obj.Q_UX(:,:,i);
            Q_u = obj.Q_U(:,i);
            Q_uu = obj.Q_UU(:,:,i) + 1e-8 * eye(obj.system.control_size);

            P = Q_uu;
            q = Q_ux*delta_x + Q_u;

            n_con = obj.system.control_size + numel(obj.constraints);
            constraint_A = zeros(n_con, obj.system.control_size);
            lb = zeros(n_con,1);
            ub = zeros(n_con,1);

            constraint_A(1:obj.system.control_size, 1:obj.system.control_size) = eye(obj.system.control_size);
            lb(1:obj.system.control_size) = -obj.system.control_bound - obj.u_trajectories(:,i);
            ub(1:obj.system.control_size) =  obj.system.control_bound - obj.u_trajectories(:,i);
            lb = lb * trust_region_scale;
            ub = ub * trust_region_scale;

            [f_x, f_u] = obj.system.transition_J(x, obj.u_trajectories(:,i));
            constraint_index = obj.system.control_size + 1;
            for j = 1:numel(obj.constraints)
                if i <= obj.horizon-1
                    x_temp = obj.system.transition(x, obj.u_trajectories(:,i));
                    D = obj.constraints{j}.evaluate_constraint(x_temp);
                    C = obj.constraints{j}.evaluate_constraint_J(x_temp) * f_u;
                    constraint_A(constraint_index, :) = C;
                    lb(constraint_index) = -Inf;
                    ub(constraint_index) = -D;
                end
                constraint_index = constraint_index + 1;
            end

            options = optimoptions('quadprog', 'Display', 'off');
            [delta_u,~,exitflag] = quadprog(P, q, [constraint_A;-constraint_A], [ub;-lb], [], [], [], [], [], options);
            if exitflag ~= 1
                feasible = false;
                trust_region_scale = trust_region_scale * 0.9;
                trust_shrink_count = trust_shrink_count + 1;
                fprintf('Trust region reduced to %e at step %d\n', trust_region_scale, i);
                if trust_shrink_count > max_trust_region_reductions
                    warning('Max trust region reductions reached, breaking out of forward pass.');
                    broke_early = true;
                    break;
                end
                broke_early = true;
                break;
            end
            u = delta_u(1:obj.system.control_size) + obj.u_trajectories(:,i);
            u_new_trajectories(:,i) = u;
            current_J = current_J + obj.system.calculate_cost(x, u);
            x = obj.system.transition(x, u);
            x_new_trajectories(:,i+1) = x;
        end
        if broke_early
            break;
        end
        current_J = current_J + obj.system.calculate_final_cost(x);
        if feasible
            obj.x_trajectories = x_new_trajectories;
            obj.u_trajectories = u_new_trajectories;
            fprintf('total cost %g\n', current_J);
        end
    end
end

        function backward_pass(obj)
            A = obj.system.Q_f;
            b = obj.system.Q_f * (obj.x_trajectories(:,obj.horizon+1) - obj.system.goal);
            for i = obj.horizon:-1:1
                u = obj.u_trajectories(:,i);
                x = obj.x_trajectories(:,i);
                l_xt = obj.system.Q * (x - obj.system.goal);
                l_ut = obj.system.R * u;
                l_uxt = zeros(obj.system.control_size, obj.system.state_size);
                l_xxt = obj.system.Q;
                l_uut = obj.system.R;
                [f_x, f_u] = obj.system.transition_J(x, u);

                Q_x = l_xt + f_x' * b;
                Q_u = l_ut + f_u' * b;
                Q_xx = l_xxt + f_x' * (A + obj.reg_factor*eye(obj.system.state_size)) * f_x;
                Q_ux = l_uxt + f_u' * (A + obj.reg_factor*eye(obj.system.state_size)) * f_x;
                Q_uu = l_uut + f_u' * (A + obj.reg_factor*eye(obj.system.state_size)) * f_u ...
                    + obj.reg_factor_u * eye(obj.system.control_size);

                Q_uu = Q_uu + 1e-8 * eye(obj.system.control_size); % extra regularization

                K = -inv(Q_uu) * Q_ux;
                k = -inv(Q_uu) * Q_u;

                A = Q_xx + K' * Q_uu * K + Q_ux' * K + K' * Q_ux;
                b = Q_x + Q_ux' * k + K' * Q_uu * k + K' * Q_u;
                obj.Q_UX(:,:,i) = Q_ux;
                obj.Q_UU(:,:,i) = Q_uu;
                obj.Q_U(:,i) = Q_u;
            end
        end

    function total_cost = compute_total_cost(obj)
        total_cost = 0;
        for t = 1:obj.horizon
            total_cost = total_cost + obj.system.calculate_cost(obj.x_trajectories(:,t), obj.u_trajectories(:,t));
        end
        total_cost = total_cost + obj.system.calculate_final_cost(obj.x_trajectories(:,obj.horizon+1));
    
        
    end
    end  
end