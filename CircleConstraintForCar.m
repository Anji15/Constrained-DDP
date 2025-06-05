classdef CircleConstraintForCar < handle
    properties
        center
        r
        system
    end
    methods
        function obj = CircleConstraintForCar(center, r, system)
            obj.center = center(:);
            obj.r = r;
            obj.system = system;
        end
        function val = evaluate_constraint(obj, x)
            x_next = obj.system.transition(x, zeros(obj.system.control_size,1));
            len = (x_next(1) - obj.center(1))^2 + (x_next(2) - obj.center(2))^2;
            val = obj.r^2 - len;
        end
        function result = evaluate_constraint_J(obj, x)
            x_next = obj.system.transition(x, zeros(obj.system.control_size,1));
            result = zeros(size(x));
            result(1) = -2*(x_next(1) - obj.center(1));
            result(2) = -2*(x_next(2) - obj.center(2));
            result(3) = -2*(x_next(1) - obj.center(1)) * obj.system.dt * x(4) * cos(x(3)) ...
                        + 2*(x_next(2) - obj.center(2)) * obj.system.dt * x(4) * sin(x(3));
            result(4) = -2*(x_next(2) - obj.center(2)) * obj.system.dt * cos(x(3)) ...
                        -2*(x_next(1) - obj.center(1)) * obj.system.dt * sin(x(3));
            result = result(:)'; % output as row for correct matrix multiply
        end
    end
end