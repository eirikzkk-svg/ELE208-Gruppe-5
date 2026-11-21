function [robot, meta] = ele208_dynamics_model(loadCase, groupRobot)
%ELE208_DYNAMICS_MODEL V2 mass/inertia and assumed shoulder viscous friction.
% [robot,meta] = ele208_dynamics_model('maximum',eirikRobot)
% Use 'empty', 'nominal' or 'maximum'. Each case includes the 2 kg gripper.
% Geometry, joint limits, base and tool are kept from the input robot.
% Without an input robot, use the saved DH model and limits.
% The gripper and object are included, so do not add them with payload().
% AI assistance with drafting and debugging the dynamics implementation,
% including load bookkeeping and motor-inertia/friction conventions.
if nargin<1 || isempty(loadCase), loadCase='maximum'; end
data=ele208_source_data();
loadCase=lower(char(loadCase));
assert(ismember(loadCase,{'empty','nominal','maximum'}), ...
    'ELE208:loadCase','Use empty, nominal or maximum. All include the 2 kg gripper.');
assert(exist('SerialLink','class')==8 && exist('Link','class')==8, ...
    'ELE208:dependency','Run setup_shoulder_friction with Peter Corke toolboxes available.');
if nargin>=2 && ~isempty(groupRobot)
    assert(isa(groupRobot,'SerialLink') && groupRobot.n==5 && groupRobot.mdh==0, ...
        'ELE208:geometry','Expected Eirik''s five standard-DH joints.');
    for i=1:5
        l=groupRobot.links(i);
        assert(l.isrevolute() && norm([l.d l.a l.alpha]-data.DH(i,:),inf)<1e-12 ...
            && abs(l.offset)<1e-12 && ~l.flip, ...
            'ELE208:geometry','Geometry differs at joint %d. Recheck body frames.',i);
    end
    assert(norm(double(groupRobot.base)-data.base,'fro')<1e-12 && ...
        norm(double(groupRobot.tool)-data.tool,'fro')<1e-12, ...
        'ELE208:frames','The adopted parameters assume the saved identity base/tool.');
    robot=SerialLink(groupRobot); % copy the robot and its links
else
    for i=1:5
        L(i)=Link('d',data.DH(i,1),'a',data.DH(i,2), ...
            'alpha',data.DH(i,3),'standard'); %#ok<AGROW>
    end
    robot=SerialLink(L,'name','ELE208 v2');
    robot.qlim=data.jointLimits;
end
selected=data.cases.(loadCase);
m=data.robot.mass; c=data.robot.com; I=data.robot.inertia;
m(5)=selected.body5.mass; c(5,:)=selected.body5.com; I(:,:,5)=selected.body5.inertia;
for i=1:5
    tensor=I(:,:,i); e=sort(eig(tensor));
    assert(m(i)>0 && all(isfinite([m(i) c(i,:) tensor(:)' ])) && ...
        norm(I(:,:,i)-I(:,:,i)','fro')<1e-12 && all(e>0) && ...
        e(1)+e(2)>=e(3)-1e-12,'ELE208:inertia','Invalid body inertia %d.',i);
    % The properties are already in DH frames and can be assigned directly.
    robot.links(i).m=m(i); robot.links(i).r=c(i,:); robot.links(i).I=I(:,:,i);
    robot.links(i).Jm=data.actuator.Jm(i);
    robot.links(i).G=data.actuator.G(i);
    robot.links(i).B=data.actuator.B(i);
    robot.links(i).Tc=data.actuator.Tc(i,:);
end
robot.gravity=data.gravityRTB;
robot.fast=false; % use the MATLAB version of RNE
robot.name=['ELE208 v2 shoulder viscous ' loadCase];
meta=struct('modelId',data.modelId,'loadCase',loadCase,'mass',m,'com',c, ...
    'inertia',I,'movingMass',sum(m),'robotOnlyMovingMass',sum(data.robot.mass), ...
    'fixedBase',data.fixedBase,'gripperMass',2,'objectMass',selected.objectMass, ...
    'actuator',data.actuator,'parameterSHA256',data.parameterSHA256);
meta.dynamicsVariantId=data.dynamicsVariantId;
meta.friction=data.friction;
meta.frameTreatment='Direct DH-body-frame COM/tensor assignment. No CAD conversion.';
meta.totalInventoryMass=meta.movingMass+meta.fixedBase.mass;
assert(abs(meta.movingMass-selected.movingMass)<1e-10,'ELE208:mass','Mass total differs.');
end
