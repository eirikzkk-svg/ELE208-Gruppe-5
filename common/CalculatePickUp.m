function T_pickup = CalculatePickUp(p_init, psi, y)
% CALCULATEPICKUP  Pick-up pose at x and z from p_init and the given y position on the conveyor.
p_pickup = [p_init(1) y p_init(3)];
T_pickup = goalPose(p_pickup, psi);
end
