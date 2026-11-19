function p = ele208_joint2_coefficients(loadCase)
%ELE208_JOINT2_COEFFICIENTS Load the shoulder model for maximum or empty load.
% p=ele208_joint2_coefficients('maximum'); % 2 kg gripper and 5 kg object
% p=ele208_joint2_coefficients('empty');   % same gripper and no object
% Keep joint2-coefficients.mat in the same folder.
% u2 is extra joint torque in N m. Angle is in rad and speed is in rad/s.
% Assumed joint viscous B=11.961697 N*m*s/rad is in the plant.
% Test pose [0 pi/4 0 0 0] is separate from trajectory home.
% Use tau2=g2(q)+u2 and hold the other joints fixed. J includes motor inertia.
% p.continuousGravity uses current-angle gravity compensation.
% p.constantHolding uses only the holding torque at the controller test pose.
if nargin<1, loadCase='maximum'; end
loadCase=lower(char(loadCase));
assert(ismember(loadCase,{'maximum','empty'}),'ELE208:loadCase','Use maximum or empty.');
file=fullfile(fileparts(mfilename('fullpath')),'joint2-coefficients.mat');
assert(isfile(file),'ELE208:data','Keep joint2-coefficients.mat beside this loader.');
s=load(file,'coefficients');
assert(strcmp(s.coefficients.dynamicsVariantId, ...
    'ELE208-v2-shoulder-viscous-2026-11-09-v1'),'ELE208:data','Wrong coefficient variant.');
p=s.coefficients.cases.(loadCase);
p.dynamicsVariantId=s.coefficients.dynamicsVariantId;
p.modelId=s.coefficients.modelId;
p.parameterSHA256=s.coefficients.parameterSHA256;
end
