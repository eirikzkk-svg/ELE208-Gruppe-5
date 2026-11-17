function q = ConstructPose(q1, q2, q3, q5)
% CONSTRUCTPOSE  Joint vector with the wrist pitch chosen so the tool points straight down.
%   q4 = -(q2 + q3) keeps the end-effector vertical.
q = [q1 q2 q3 -(q2+q3) q5];
end
