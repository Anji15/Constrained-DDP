classdef Car < handle
    properties
        state_size
        control_size
        dt
        control_bound
        goal
        Q
        R
        Q_f
    end
    methods
        function obj = Car()
            obj.state_size = 4;
            obj.control_size = 2;
            obj.dt = 0.05;
            obj.control_bound = [pi/2; 10];
            obj.goal = zeros(4,1);
            obj.Q = zeros(4,4);
            obj.R = 0.05*eye(2);
            obj.Q_f = eye(4);
        end
        function set_cost(obj, Q, R)
            obj.Q = Q;
            obj.R = R;
        end
        function set_final_cost(obj, Q_f)
            obj.Q_f = Q_f;
        end
        function set_goal(obj, x_goal)
            obj.goal = x_goal;
        end
        function c = calculate_cost(obj, x, u)
            c = 0.5*((x-obj.goal)'*obj.Q*(x-obj.goal) + u'*obj.R*u);
        end
        function c = calculate_final_cost(obj, x)
            c = 0.5*(x-obj.goal)'*obj.Q_f*(x-obj.goal);
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
            figure(1); clf; hold on;
            rectangle('Position',[1-0.5,1-0.5,1,1],'Curvature',[1 1],'FaceColor',[0 0.8 0.8],'EdgeColor','none');
            rectangle('Position',[2-1,2-1,2,2],'Curvature',[1 1],'FaceColor',[0 0.8 0.8],'EdgeColor','none');
            plot(x_trajectories(1,:), x_trajectories(2,:), 'b.-','LineWidth',1.5);
            for i = 1:5:(size(x_trajectories,2)-1)
                rectangle('Position',[x_trajectories(1,i)-0.1,x_trajectories(2,i)-0.1,0.2,0.2],'Curvature',[1 1],'EdgeColor','k');
                quiver(x_trajectories(1,i), x_trajectories(2,i), 0.1*sin(x_trajectories(3,i)), 0.1*cos(x_trajectories(3,i)), 0, 'k','MaxHeadSize',1);
            end
            plot(obj.goal(1), obj.goal(2), 'rx', 'MarkerSize',12,'LineWidth',2);
            axis equal; xlim([-1,4]); ylim([-1,4]);
            title('Car Trajectory');
            hold off;
            drawnow;
        end
    end
end