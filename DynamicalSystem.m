classdef DynamicalSystem < handle
    properties
        state_size
        control_size
        Q
        R
        Q_f
        goal
    end
    methods
        function obj = DynamicalSystem(state_size, control_size)
            obj.state_size = state_size;
            obj.control_size = control_size;
        end
        function set_cost(obj, Q, R)
            obj.Q = Q;
            obj.R = R;
        end
        function set_final_cost(obj, Q_f)
            obj.Q_f = Q_f;
        end
        function c = calculate_cost(obj, x, u)
            % one step cost = 0.5*((x-goal)'*Q*(x-goal) + u'*R*u)
            c = 0.5*((x-obj.goal)'*obj.Q*(x-obj.goal) + u'*obj.R*u);
        end
        function c = calculate_final_cost(obj, x)
            c = 0.5*(x-obj.goal)'*obj.Q_f*(x-obj.goal);
        end
        function set_goal(obj, x_goal)
            obj.goal = x_goal;
        end
    end
end

classdef DoubleIntegrator < DynamicalSystem
    properties
        dt
        control_bound
    end
    methods
        function obj = DoubleIntegrator()
            obj@DynamicalSystem(4, 2);
            obj.dt = 0.05;
            obj.control_bound = ones(2,1) * 100;
            obj.goal = zeros(4,1);
        end
        function result = transition(obj, x, u)
            result = zeros(4,1);
            result(1:2) = x(1:2) + obj.dt * x(3:4);
            result(3:4) = x(3:4) + obj.dt * u;
        end
        function [A,B] = transition_J(obj, x, u)
            A = eye(obj.state_size);
            B = zeros(obj.state_size, obj.control_size);
            A(1,3) = obj.dt;
            A(2,4) = obj.dt;
            B(3,1) = obj.dt;
            B(4,2) = obj.dt;
        end
        function draw_trajectories(obj, x_trajectories)
            figure; ax = subplot(111);
            circle1 = rectangle('Position',[1-0.5,1-0.5,1,1],'Curvature',[1 1],'FaceColor',[0 0.8 0.8],'EdgeColor','none');
            hold on;
            circle2 = rectangle('Position',[1.5-0.5,2.2-0.5,1,1],'Curvature',[1 1],'FaceColor',[0 0.8 0.8],'EdgeColor','none');
            scatter(x_trajectories(1,1:5:end), x_trajectories(2,1:5:end), 4,'r','filled');
            axis equal; xlim([0,3]); ylim([0,3]);
            hold off;
        end
        function draw_u_trajectories(obj, u_trajectories)
            figure;
            scatter(u_trajectories(1,1:5:end), u_trajectories(2,1:5:end), 4,'r','filled');
        end
    end
end

classdef Car < DynamicalSystem
    properties
        dt
        control_bound
    end
    methods
        function obj = Car()
            obj@DynamicalSystem(4,2);
            obj.dt = 0.05;
            obj.control_bound = [pi/2; 10];
            obj.goal = zeros(4,1);
        end
        function x_next = transition(obj, x, u)
            x_next = zeros(4,1);
            x_next(1) = x(1) + obj.dt * x(4) * sin(x(3));
            x_next(2) = x(2) + obj.dt * x(4) * cos(x(3));
            x_next(3) = x(3) + obj.dt * u(2) * x(4);
            x_next(4) = x(4) + obj.dt * u(1);
        end
        function [A, B] = transition_J(obj, x, u)
            A = eye(4);
            B = zeros(4,2);
            A(1,4) = sin(x(3)) * obj.dt;
            A(1,3) = x(4) * cos(x(3)) * obj.dt;
            A(2,4) = cos(x(3)) * obj.dt;
            A(2,3) = -x(4) * sin(x(3)) * obj.dt;
            A(3,4) = u(2) * obj.dt;
            B(3,2) = x(4) * obj.dt;
            B(4,1) = obj.dt;
        end
        function draw_trajectories(obj, x_trajectories)
            figure; ax = subplot(111);
            rectangle('Position',[1-0.5,1-0.5,1,1],'Curvature',[1 1],'FaceColor',[0 0.8 0.8],'EdgeColor','none'); hold on;
            rectangle('Position',[2-1,2-1,2,2],'Curvature',[1 1],'FaceColor',[0 0.8 0.8],'EdgeColor','none');
            for i = 1:5:(size(x_trajectories,2)-1)
                rectangle('Position',[x_trajectories(1,i)-0.1,x_trajectories(2,i)-0.1,0.2,0.2],'Curvature',[1 1],'EdgeColor','k');
                quiver(x_trajectories(1,i), x_trajectories(2,i), 0.1*sin(x_trajectories(3,i)), 0.1*cos(x_trajectories(3,i)), 0, 'k','MaxHeadSize',1);
            end
            axis equal; xlim([-1,4]); ylim([-1,4]);
            hold off;
        end
    end
end

classdef Quadrotor < DynamicalSystem
    properties
        dt
        control_bound
    end
    methods
        function obj = Quadrotor()
            obj@DynamicalSystem(12,4);
            obj.dt = 0.02;
            obj.control_bound = [10;10;10;10];
            obj.goal = zeros(12,1);
        end
        function x_next = transition(obj, x, u)
            forces = [0;0;sum(u)];
            torques = [u(1)-u(3); u(2)-u(4); u(1)-u(2)+u(3)-u(4)];
            rotation_matrix = obj.get_rotation_matrix(x);
            J_omega = obj.get_J_omega(x);
            g = [0;0;-10];
            x_next = zeros(12,1);
            x_next(1:3) = x(1:3) + obj.dt * x(7:9);
            x_next(7:9) = x(7:9) + obj.dt * (g + rotation_matrix*forces);
            x_next(4:6) = x(4:6) + obj.dt * J_omega * x(10:12);
            x_next(10:12) = x(10:12) + obj.dt * torques;
        end
        function [A, B] = transition_J(obj, x, u)
            A = zeros(obj.state_size, obj.state_size);
            B = zeros(obj.state_size, obj.control_size);
            u_sum = sum(u);
            rotation_matrix = obj.get_rotation_matrix(x);
            A(1:obj.state_size,1:obj.state_size) = eye(obj.state_size);
            A(1,7) = 1 * obj.dt;
            A(2,8) = 1 * obj.dt;
            A(3,9) = 1 * obj.dt;
            % Only partial derivatives are implemented as in the Python code
            A(7,4) = A(7,4) + u_sum * obj.dt * (sin(x(5))*cos(x(4)));
            A(7,5) = A(7,5) + u_sum * obj.dt * (cos(x(5))*sin(x(4)));
            A(8,5) = A(8,5) + u_sum * obj.dt * (-cos(x(6))*cos(x(5)));
            A(8,6) = A(8,6) + u_sum * obj.dt * (sin(x(6))*sin(x(5)));
            A(9,5) = A(9,5) + u_sum * obj.dt * (-sin(x(5)));
            % Attitude derivatives (Euler angles)
            A(4,4) = A(4,4) + (x(11)*cos(x(4))*tan(x(5)) - x(12)*sin(x(4))*tan(x(5))) * obj.dt;
            A(4,5) = A(4,5) + obj.dt * 1.0/(cos(x(5))^2) * (x(11)*sin(x(4)) + x(12)*cos(x(4)));
            A(4,10) = A(4,10) + obj.dt;
            A(4,11) = A(4,11) + sin(x(4))*tan(x(5))*obj.dt;
            A(4,12) = A(4,12) + cos(x(4))*tan(x(5))*obj.dt;
            A(5,4) = A(5,4) + obj.dt * (-sin(x(4))*x(11) - cos(x(4))*x(12));
            A(5,11) = A(5,11) + obj.dt * cos(x(4));
            A(5,12) = A(5,12) - obj.dt * sin(x(4));
            A(6,4) = A(6,4) + obj.dt * (cos(x(4))/cos(x(5))*x(11) - sin(x(4))/cos(x(5))*x(12));
            A(6,5) = A(6,5) + obj.dt * sin(x(5))/(cos(x(5))^2)*(sin(x(4))*x(11) + cos(x(4))*x(12));
            A(6,11) = A(6,11) + obj.dt * sin(x(4))/cos(x(5));
            A(6,12) = A(6,12) + obj.dt * cos(x(4))/cos(x(5));
            torque_matrix = zeros(3,4);
            torque_matrix(1,1) = 1; torque_matrix(1,3) = -1;
            torque_matrix(2,2) = 1; torque_matrix(2,4) = -1;
            torque_matrix(3,1) = 1; torque_matrix(3,2) = -1; torque_matrix(3,3) = 1; torque_matrix(3,4) = -1;
            force_matrix = zeros(3,4); force_matrix(3,:) = 1;
            B(10:12,:) = torque_matrix * obj.dt;
            B(7:9,:) = obj.dt * rotation_matrix * force_matrix;
        end
        function R = get_rotation_matrix(obj, x)
            R = zeros(3,3);
            % Matlab angles are 1-based: x(4)=phi, x(5)=theta, x(6)=psi
            R(1,1) = cos(x(4))*cos(x(6)) - cos(x(5))*sin(x(4))*sin(x(6));
            R(1,2) = -cos(x(4))*sin(x(4)) - cos(x(4))*cos(x(5))*sin(x(6));
            R(1,3) = sin(x(5))*sin(x(4));
            R(2,1) = cos(x(5))*cos(x(6))*sin(x(4));
            R(2,2) = cos(x(4))*cos(x(5))*cos(x(6)) - sin(x(4))*sin(x(6));
            R(2,3) = -cos(x(6))*sin(x(5));
            R(3,1) = sin(x(4))*sin(x(5));
            R(3,2) = cos(x(4))*sin(x(5));
            R(3,3) = cos(x(5));
        end
        function J_omega = get_J_omega(obj, x)
            J_omega = zeros(3,3);
            J_omega(1,1) = 1;
            J_omega(1,2) = sin(x(4))*tan(x(5));
            J_omega(1,3) = cos(x(4))*tan(x(5));
            J_omega(2,2) = cos(x(4));
            J_omega(2,3) = -sin(x(4));
            J_omega(3,2) = sin(x(4))/cos(x(5));
            J_omega(3,3) = cos(x(4))/cos(x(5));
        end
    end
end

% Example usage (main script):
if ~isdeployed && ~ismcc
    system = Quadrotor();
    initial_state = [-3.5; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
    initial_u = [2.5; 2.5; 2.5; 2.5];
    perturb_u = [0.01; 0; 0; 0];
    perturb_x = zeros(12,1);
    perturb_x(5) = 0.01;
    [A,B] = system.transition_J(initial_state, initial_u);
    next_state = system.transition(initial_state + perturb_x, initial_u + perturb_u);
    linearized_next_state = system.transition(initial_state, initial_u) + A * perturb_x + B * perturb_u;
    disp(next_state - linearized_next_state);
end