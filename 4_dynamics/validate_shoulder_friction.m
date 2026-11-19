function results = validate_shoulder_friction(action,outFolder)
%VALIDATE_SHOULDER_FRICTION Existing saved-linear/full-RNE shoulder comparison.
% Writes results/shoulder; optional 'figures' redraws its saved output only.
packageRoot=fileparts(mfilename('fullpath'));
if nargin<1,action='';end
if nargin<2 || isempty(outFolder),outFolder=fullfile(packageRoot,'results','shoulder');end
here=outFolder;
if ~isfolder(here),mkdir(here);end
if ~isempty(action)
    assert(strcmp(action,'figures'),'The optional action is figures.');
    saved=load(fullfile(here,'numerical-results.mat'));
    makeFigures(here,saved.runs,saved.sweep); results=saved.results;
    return;
end
oldPath=path; runTimer=tic;
restorePath=onCleanup(@()path(oldPath));
modelFolder=packageRoot;
addpath(modelFolder,'-begin');
environment=setup_shoulder_friction();
groupFolder=fullfile(packageRoot,'support');
addpath(groupFolder,'-begin');
for name={'ele208_source_data','ele208_dynamics_model','ele208_joint2_coefficients','setup_shoulder_friction'}
    assert(strcmp(which(name{1}),fullfile(modelFolder,[name{1} '.m'])), ...
        'Wrong model or coefficient path: %s',name{1});
end
assert(strcmp(which('createRobot'),fullfile(groupFolder,'createRobot.m')));
environment.createRobot=which('createRobot');
environment.coefficients=which('ele208_joint2_coefficients');
environment.coefficientMAT=fullfile(modelFolder,'joint2-coefficients.mat');
environment.rne=fullfile(fileparts(which('SerialLink')),'rne.m');
environment.validation=mfilename('fullpath');
environment.dependencies=jsondecode(fileread(fullfile(packageRoot,'external-toolboxes.json')));
environment.controlSystemToolboxUsed=false;
[original,T0C]=createRobot(); originalLimits=original.qlim;
data=ele208_source_data(); q0=[0 pi/4 0 0 0];
assert(strcmp(data.modelId,'ELE208-hollow-modular-2026-11-04-v2'));
assert(strcmp(data.dynamicsVariantId,'ELE208-v2-shoulder-viscous-2026-11-09-v1'));
preflight=jsondecode(fileread(fullfile(packageRoot,'data','expected-results.json')));
assert(strcmp(preflight.status,'PASS') && strcmp(preflight.parameterSHA256,data.parameterSHA256));
fprintf('Resolved model: %s\nResolved coefficients: %s\nRNE: %s\n', ...
    environment.model,environment.coefficientMAT,environment.rne);
settings=struct('solver','ode45','RelTol',1e-9,'AbsTol',[1e-11 1e-11], ...
    'MaxStep_s',0.05,'outputInterval_s',0.01,'initialState_rad_rad_s',[0 0], ...
    'stepAmplitude_Nm',0.5,'continuousDuration_s',8,'holdingDuration_s',2.5, ...
    'smallWindow_s',1,'limitEvent','terminal at either shoulder limit', ...
    'linearPrediction','exact matrix exponential of saved state matrices for constant step', ...
    'gravityContinuous','tau2 = current correct-load g2(q) + 0.5 N m', ...
    'gravityHolding','tau2 = saved g2(q0) + 0.5 N m', ...
    'rmsDefinition','sqrt(mean(sampled error squared)); relative error normalized by same-window linear peak');
results=struct('status','RUNNING','modelId',data.modelId, ...
    'dynamicsVariantId',data.dynamicsVariantId,'parameterSHA256',data.parameterSHA256, ...
    'executedAtUTC',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
    'environment',environment,'settings',settings,'q0_rad',q0, ...
    'heldJoints',[1 3 4 5],'jointLimits_rad',original.qlim);
fprintf('ELE208 friction 4.ii validation: %s\nMATLAB %s / %s\n',data.dynamicsVariantId,version,computer);
fprintf('Step +%.3g N m, zero initial deviation/speed. Other joints HELD.\n',settings.stepAmplitude_Nm);
names={'maximum','empty'}; modes={'continuous','holding'};
runs=struct(); sweep=struct();
for ci=1:2
    name=names{ci}; [robot,meta]=ele208_dynamics_model(name,original);
    p=ele208_joint2_coefficients(name);
    assert(strcmp(p.modelId,data.modelId) && strcmp(p.parameterSHA256,data.parameterSHA256));
    assert(isequal(p.q0_rad,q0) && isequal(p.otherJointsHeld,[1 3 4 5]));
    M0=robot.inertia(q0); g0=robot.gravload(q0);
    assert(strcmp(p.dynamicsVariantId,data.dynamicsVariantId) && strcmp(meta.dynamicsVariantId,data.dynamicsVariantId));
    assert(abs(M0(2,2)-p.J_kgm2)<1e-10 && abs(g0(2)-p.tauEquilibrium_Nm)<1e-10);
    assert(isequal([robot.links.B],data.actuator.B) && ...
        all(vertcat(robot.links.Tc)==0,'all') && ~robot.fast);
    assert(abs(robot.links(2).B*robot.links(2).G^2-p.B_Nm_s_rad)<1e-12);
    assert(abs(robot.links(2).Jm*robot.links(2).G^2-p.JrotorAlreadyIncluded_kgm2)<1e-12);
    results.savedModelChecks.(name)=struct('J_kgm2',M0(2,2), ...
        'tau0_Nm',g0(2),'Kg_Nm_rad',p.Kg_Nm_rad,'B_Nm_s_rad',p.B_Nm_s_rad, ...
        'rotorAlreadyIncluded_kgm2',p.JrotorAlreadyIncluded_kgm2, ...
        'gripperMass_kg',meta.gripperMass,'objectMass_kg',meta.objectMass, ...
        'movingMass_kg',meta.movingMass,'linearModel',p);
    for mi=1:2
        mode=modes{mi};
        if mi==1, duration=settings.continuousDuration_s; else, duration=settings.holdingDuration_s; end
        t=(0:settings.outputInterval_s:duration)';
        [yn,solverInfo]=integrateRobot(robot,q0,p,mode,settings,t);
        yl=linearStep(p,mode,settings.stepAmplitude_Nm,t);
        q= q0+yn(:,1)*[0 1 0 0 0]; qd=yn(:,2)*[0 1 0 0 0];
        n=numel(t); zero=zeros(n,5); e2=repmat([0 1 0 0 0],n,1);
        bias=robot.rne(q,qd,zero); unit=robot.rne(q,qd,e2);
        J=unit(:,2)-bias(:,2); gravity=robot.gravload(q);
        if mi==1, tau2=gravity(:,2)+settings.stepAmplitude_Nm;
        else, tau2=p.tauEquilibrium_Nm+settings.stepAmplitude_Nm+zeros(n,1); end
        a=(tau2-bias(:,2))./J;
        % Reaction/holding torques from the FULL inverse dynamics, not zeros.
        required=robot.rne(q,qd,a*[0 1 0 0 0]);
        shoulderResidual=max(abs(required(:,2)-tau2));
        ids=unique(round(linspace(1,n,11))); heldResidual=0; accelerationResidual=0;
        for k=ids
            % Full forward-dynamics recovery using all five required torques.
            actual=robot.accel(q(k,:),qd(k,:),required(k,:));
            heldResidual=max(heldResidual,max(abs(actual([1 3 4 5]))));
            accelerationResidual=max(accelerationResidual,abs(actual(2)-a(k)));
        end
        assert(shoulderResidual<1e-9 && heldResidual<1e-9 && accelerationResidual<1e-9);
        allAngles=[q; q0+yl(:,1)*[0 1 0 0 0]];
        assert(all(allAngles>=original.qlim(:,1)' & allAngles<=original.qlim(:,2)','all'));
        small=t<=settings.smallWindow_s+eps;
        rem=gravity(:,2)-p.tauEquilibrium_Nm-p.Kg_Nm_rad*yn(:,1);
        summary=struct('loadCase',name,'gravityMode',mode, ...
            'amplitude_Nm',settings.stepAmplitude_Nm,'duration_s',duration, ...
            'solverInfo',solverInfo,'smallWindow',errorMetrics(yn(small,:),yl(small,:)), ...
            'fullWindow',errorMetrics(yn,yl), ...
            'minimumJointLimitMargin_rad',min(min(allAngles-original.qlim(:,1)',original.qlim(:,2)'-allAngles),[],'all'), ...
            'maxGravityTaylorRemainder_Nm',max(abs(rem)), ...
            'JRangeRNE_kgm2',[min(J) max(J)], ...
            'maxJDepartureFromSaved_kgm2',max(abs(J-p.J_kgm2)), ...
            'maxBiasMinusGravityAndViscous_Nm',max(abs(bias(:,2)-gravity(:,2)-p.B_Nm_s_rad*yn(:,2))), ...
            'maxViscousTorque_Nm',max(abs(p.B_Nm_s_rad*yn(:,2))), ...
            'finalNonlinearState_rad_rad_s',yn(end,:), ...
            'finalLinearState_rad_rad_s',yl(end,:), ...
            'shoulderRangeNonlinear_rad',[min(q(:,2)) max(q(:,2))], ...
            'shoulderRangeLinear_rad',[min(q0(2)+yl(:,1)) max(q0(2)+yl(:,1))], ...
            'maxShoulderEquationResidual_Nm',shoulderResidual, ...
            'forwardDynamicsSamples',numel(ids),'maxHeldJointAcceleration_rad_s2',heldResidual, ...
            'maxShoulderAccelerationRecoveryError_rad_s2',accelerationResidual, ...
            'heldReactionMin_Nm',min(required(:,[1 3 4 5]),[],1), ...
            'heldReactionMax_Nm',max(required(:,[1 3 4 5]),[],1));
        if mi==1
            assert(summary.fullWindow.maxAngleError_rad<1e-8 && summary.fullWindow.maxSpeedError_rad_s<1e-8);
        end
        runs.(name).(mode)=struct('t',t,'nonlinear',yn,'linear',yl, ...
            'input_Nm',settings.stepAmplitude_Nm+zeros(n,1),'tau2_Nm',tau2, ...
            'reaction_Nm',required(:,[1 3 4 5]),'gravityRemainder_Nm',rem,'summary',summary);
        results.responses.(name).(mode)=summary;
        T=table(t,yn(:,1),yl(:,1),yn(:,2),yl(:,2),yn(:,1)-yl(:,1),yn(:,2)-yl(:,2), ...
            settings.stepAmplitude_Nm+zeros(n,1),tau2,required(:,1),required(:,3),required(:,4),required(:,5),rem, ...
            'VariableNames',{'time_s','x_nonlinear_rad','x_linear_rad','speed_nonlinear_rad_s','speed_linear_rad_s', ...
            'angle_error_rad','speed_error_rad_s','perturbation_Nm','total_shoulder_torque_Nm', ...
            'hold_joint1_Nm','hold_joint3_Nm','hold_joint4_Nm','hold_joint5_Nm','gravity_taylor_remainder_Nm'});
        writetable(T,fullfile(here,[name '-' mode '-response.csv']));
        fprintf('%s %s %.1f s: nonlinear angle %.6f deg, speed %.6f deg/s; max errors %.3g rad / %.3g rad/s\n', ...
            name,mode,duration,rad2deg(yn(end,1)),rad2deg(yn(end,2)), ...
            summary.fullWindow.maxAngleError_rad,summary.fullWindow.maxSpeedError_rad_s);
    end
    % Gravity curve at this test pose, holding the straight elbow fixed.
    x=deg2rad(linspace(-30,35,261))'; Q=q0+x*[0 1 0 0 0];
    G=robot.gravload(Q); tangent=p.tauEquilibrium_Nm+p.Kg_Nm_rad*x;
    remainder=G(:,2)-tangent;
    harmonic=p.tauEquilibrium_Nm*cos(x)+p.Kg_Nm_rad*sin(x);
    assert(max(abs(G(:,2)-harmonic))<1e-9);
    sweep.(name)=struct('x_rad',x,'gravity_Nm',G(:,2),'tangent_Nm',tangent,'remainder_Nm',remainder);
    writetable(table(x,G(:,2),tangent,remainder,'VariableNames', ...
        {'displacement_rad','gravity_Nm','tangent_Nm','remainder_Nm'}),fullfile(here,[name '-gravity-sweep.csv']));
    xp=deg2rad([-30 -15 -5 5 15 30])'; Q=q0+xp*[0 1 0 0 0]; Gp=robot.gravload(Q);
    results.gravityLinearity.(name)=struct('displacement_deg',rad2deg(xp), ...
        'exactGravityChange_Nm',Gp(:,2)-p.tauEquilibrium_Nm, ...
        'linearGravityChange_Nm',p.Kg_Nm_rad*xp, ...
        'TaylorRemainder_Nm',Gp(:,2)-p.tauEquilibrium_Nm-p.Kg_Nm_rad*xp, ...
        'harmonicIdentityResidual_Nm',max(abs(G(:,2)-harmonic)));
end
% One convergence rerun: empty-load constant holding has the largest departure.
robot=ele208_dynamics_model('empty',original); p=ele208_joint2_coefficients('empty');
tighter=settings; tighter.RelTol=1e-11; tighter.AbsTol=[1e-13 1e-13]; tighter.MaxStep_s=0.025;
results.convergenceSettings=tighter;
r=runs.empty.holding;
[yt,info]=integrateRobot(robot,q0,p,'holding',tighter,r.t);
difference=yt-r.nonlinear;
results.convergence=struct('loadCase','empty','mode','holding','solverInfo',info, ...
    'maxAngleChange_rad',max(abs(difference(:,1))), ...
    'maxSpeedChange_rad_s',max(abs(difference(:,2))), ...
    'angleChangeRelativeToModelError',max(abs(difference(:,1)))/r.summary.fullWindow.maxAngleError_rad, ...
    'speedChangeRelativeToModelError',max(abs(difference(:,2)))/r.summary.fullWindow.maxSpeedError_rad_s);
assert(max(abs(difference),[],'all')<1e-8);
convergence=struct('t',r.t,'tighter',yt,'baseline',r.nonlinear,'difference',difference);
writetable(table(r.t,yt(:,1),yt(:,2),difference(:,1),difference(:,2), ...
    'VariableNames',{'time_s','tighter_angle_rad','tighter_speed_rad_s','angle_change_rad','speed_change_rad_s'}), ...
    fullfile(here,'solver-convergence.csv'));
assert(isequal(original.qlim,originalLimits) && all([original.links.Jm]==0) && all([original.links.B]==0));
[originalAgain,T0CAgain]=createRobot();
assert(norm(double(T0C)-double(T0CAgain),'fro')==0 && ...
    norm(double(original.fkine(q0))-double(originalAgain.fkine(q0)),'fro')==0);
makeFigures(here,runs,sweep);
results.status='PASS';
results.completedAtUTC=char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX'));
results.elapsed_s=toc(runTimer);
results.scope='Held-joint mathematical model validation only. No controller, actuator or pick-and-place performance claim.';
writeJSON(fullfile(here,'numerical-results.json'),results);
save(fullfile(here,'numerical-results.mat'),'results','runs','sweep','convergence');
fprintf('PASS: four current-state RNE responses, one tighter solver rerun, reaction torques and gravity curves.\n');
end

function [y,info]=integrateRobot(robot,q0,p,mode,settings,t)
opts=odeset('RelTol',settings.RelTol,'AbsTol',settings.AbsTol, ...
    'MaxStep',settings.MaxStep_s,'Events',@(t,y)limits(t,y,q0,robot.qlim));
% Integrate ONLY the free coordinate. All four held coordinates have exact
% prescribed q, qd=0, qdd=0. Full RNE supplies their reaction torques later.
sol=ode45(@(t,y)robotRHS(t,y,robot,q0,p,mode,settings.stepAmplitude_Nm), ...
    [t(1) t(end)],settings.initialState_rad_rad_s(:),opts);
assert(abs(sol.x(end)-t(end))<1e-12,'A joint limit stopped the run. Shorten the stated test.');
y=deval(sol,t)';
info=struct('internalAcceptedSteps',numel(sol.x)-1,'reachedTime_s',sol.x(end), ...
    'outputSamples',numel(t),'limitEventTriggered',~isempty(sol.xe), ...
    'minimumAcceptedShoulderLimitMargin_rad',min([q0(2)+sol.y(1,:)-robot.qlim(2,1),robot.qlim(2,2)-q0(2)-sol.y(1,:)]));
end

function dy=robotRHS(~,y,robot,q0,p,mode,u)
q=q0; q(2)=q0(2)+y(1); qd=[0 y(2) 0 0 0];
% NONLINEAR dynamics are evaluated at every current state. No saved J or Kg
% is used in the nonlinear acceleration. RNE includes all bodies, rotor and adopted viscous friction.
b=robot.rne(q,qd,zeros(1,5)); r=robot.rne(q,qd,[0 1 0 0 0]);
J=r(2)-b(2); assert(J>0);
if strcmp(mode,'continuous')
    g=robot.gravload(q); tau=g(2)+u;
else
    tau=p.tauEquilibrium_Nm+u;
end
dy=[y(2);(tau-b(2))/J];
end

function [value,terminal,direction]=limits(~,y,q0,qlim)
q2=q0(2)+y(1);
value=[q2-qlim(2,1);qlim(2,2)-q2]; terminal=[1;1]; direction=[-1;-1];
end

function y=linearStep(p,mode,u,t)
% The independent comparison uses the SAVED linear state matrices, zero ICs.
if strcmp(mode,'continuous'), plant=p.continuousGravity; else, plant=p.constantHolding; end
augmented=[plant.A plant.B*u; zeros(1,3)];
y=zeros(numel(t),2);
for k=1:numel(t)
    transition=expm(augmented*t(k));
    y(k,:)=(plant.C*transition(1:2,3)+plant.D*u)';
end
end

function m=errorMetrics(n,l)
e=n-l;
m=struct('peakNonlinearAngle_rad',max(abs(n(:,1))), ...
    'peakLinearAngle_rad',max(abs(l(:,1))), ...
    'peakNonlinearSpeed_rad_s',max(abs(n(:,2))), ...
    'peakLinearSpeed_rad_s',max(abs(l(:,2))), ...
    'maxAngleError_rad',max(abs(e(:,1))),'rmsAngleError_rad',sqrt(mean(e(:,1).^2)), ...
    'maxSpeedError_rad_s',max(abs(e(:,2))),'rmsSpeedError_rad_s',sqrt(mean(e(:,2).^2)), ...
    'angleError_percentOfLinearPeak',100*max(abs(e(:,1)))/max(abs(l(:,1))), ...
    'speedError_percentOfLinearPeak',100*max(abs(e(:,2)))/max(abs(l(:,2))));
end

function makeFigures(here,runs,sweep)
names={'maximum','empty'}; colours=[0 0.447 0.741;0.85 0.325 0.098];
f=figure('Visible','off','Color','w','Position',[100 100 1100 760]);
closeF=onCleanup(@()close(f));
tl=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
title(tl,{'Current-angle correct-load gravity compensation | +0.5 N m step', ...
    'Viscous-friction variant | q0 = [0, pi/4, 0, 0, 0] | held joints 1, 3, 4, 5'});
for state=1:2
    ax=nexttile(tl,(state-1)*2+1); hold(ax,'on');
    for ci=1:2
        r=runs.(names{ci}).continuous;
        plot(ax,r.t,rad2deg(r.nonlinear(:,state)),'Color',colours(ci,:),'LineWidth',2, ...
            'DisplayName',[names{ci} ' nonlinear robot']);
        plot(ax,r.t,rad2deg(r.linear(:,state)),'--','Color',colours(ci,:),'LineWidth',1.4, ...
            'DisplayName',[names{ci} ' transfer function']);
    end
    if state==1, ylabel(ax,'Shoulder displacement x (deg)'); else, ylabel(ax,'Shoulder speed (deg/s)'); end
    xlabel(ax,'Time (s)'); grid(ax,'on'); legend(ax,'Location','northwest');
    ax=nexttile(tl,state*2); hold(ax,'on');
    for ci=1:2
        r=runs.(names{ci}).continuous;
        plot(ax,r.t,1e13*(r.nonlinear(:,state)-r.linear(:,state)), ...
            'Color',colours(ci,:),'LineWidth',1.5,'DisplayName',names{ci});
    end
    if state==1, ylabel(ax,'Angle error (10^{-13} rad)'); else, ylabel(ax,'Speed error (10^{-13} rad/s)'); end
    title(ax,'RNE minus saved model (numerical error)'); xlabel(ax,'Time (s)'); grid(ax,'on'); legend(ax,'Location','best');
end
exportFigure(f,fullfile(here,'01-continuous-gravity.png'));
clear closeF;
f=figure('Visible','off','Color','w','Position',[100 100 1100 760]); closeF=onCleanup(@()close(f));
tl=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
title(tl,{'Constant equilibrium holding torque + 0.5 N m', ...
    'Viscous-friction variant | q0 = [0, pi/4, 0, 0, 0] | held joints 1, 3, 4, 5'});
for ci=1:2
    r=runs.(names{ci}).holding;
    for state=1:2
        ax=nexttile(tl,(state-1)*2+ci);
        plot(ax,r.t,rad2deg(r.nonlinear(:,state)),'Color',colours(ci,:),'LineWidth',2); hold(ax,'on');
        plot(ax,r.t,rad2deg(r.linear(:,state)),'k--','LineWidth',1.4);
        xline(ax,1,':','1 s small window','LabelVerticalAlignment','middle');
        title(ax,[names{ci} ' object load, same 2 kg gripper']); xlabel(ax,'Time (s)'); grid(ax,'on');
        if state==1, ylabel(ax,'Shoulder displacement x (deg)'); else, ylabel(ax,'Shoulder speed (deg/s)'); end
        legend(ax,'Nonlinear robot','Local transfer function','Location','northwest');
    end
end
exportFigure(f,fullfile(here,'02-constant-holding.png'));
clear closeF;
f=figure('Visible','off','Color','w','Position',[100 100 1100 760]); closeF=onCleanup(@()close(f));
tl=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
title(tl,{'Gravity linearisation at q0 = [0, pi/4, 0, 0, 0]', ...
    'Straight elbow held fixed | both loads retain 2 kg gripper | sampled angles within limits'});
for ci=1:2
    s=sweep.(names{ci}); ax=nexttile(tl,ci);
    plot(ax,rad2deg(s.x_rad),s.gravity_Nm,'Color',colours(ci,:),'LineWidth',2); hold(ax,'on');
    plot(ax,rad2deg(s.x_rad),s.tangent_Nm,'k--','LineWidth',1.4);
    title(ax,[names{ci} ' object load, same 2 kg gripper']); xlabel(ax,'Displacement from q0 (deg)'); ylabel(ax,'Holding torque (N m)');
    grid(ax,'on'); legend(ax,'Robot gravity','Equilibrium tangent','Location','southwest');
    ax=nexttile(tl,ci+2);
    plot(ax,rad2deg(s.x_rad),s.remainder_Nm,'Color',colours(ci,:),'LineWidth',2);
    xlabel(ax,'Displacement from q0 (deg)'); ylabel(ax,'Exact gravity minus tangent (N m)'); grid(ax,'on');
end
exportFigure(f,fullfile(here,'03-gravity-linearisation.png'));
clear closeF;
f=figure('Visible','off','Color','w','Position',[100 100 1100 760]); closeF=onCleanup(@()close(f));
tl=tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
title(tl,{'Applied torques required to hold joints 1, 3, 4, 5 fixed', ...
    'Full RNE reaction torques | +0.5 N m shoulder step | same 2 kg gripper'});
held=[1 3 4 5];
for ci=1:2
    for mi=1:2
        modes={'continuous','holding'}; mode=modes{mi};
        r=runs.(names{ci}).(mode); ax=nexttile(tl,(mi-1)*2+ci);
        plot(ax,r.t,r.reaction_Nm,'LineWidth',1.5);
        title(ax,[names{ci} ' / ' mode]); xlabel(ax,'Time (s)');
        ylabel(ax,'Required applied holding torque (N m)'); grid(ax,'on');
        legend(ax,compose('Joint %d',held),'Location','best');
    end
end
exportFigure(f,fullfile(here,'04-held-joint-reactions.png'));
end

function exportFigure(f,file)
% Make exports legible regardless of the user's MATLAB desktop theme.
if isprop(f,'Theme'), f.Theme='light'; end
set(findall(f,'Type','axes'),'Color','w','XColor',[0.15 0.15 0.15], ...
    'YColor',[0.15 0.15 0.15],'GridColor',[0.65 0.65 0.65],'FontSize',11);
set(findall(f,'Type','text'),'Color',[0.1 0.1 0.1]);
set(findall(f,'Type','legend'),'Color','w','TextColor',[0.1 0.1 0.1], ...
    'EdgeColor',[0.5 0.5 0.5]);
exportgraphics(f,file,'Resolution',180,'BackgroundColor','white');
end

function writeJSON(file,value)
fid=fopen(file,'w'); assert(fid~=-1); finish=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end
