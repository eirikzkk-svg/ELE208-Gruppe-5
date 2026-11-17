function T = goalPose(p, psi)
% GOALPOSE  End-effector pose at position p, pointing down, rotated psi about the vertical axis.
T = SE3(transl(p) * trotz(psi) * trotx(pi));
end
