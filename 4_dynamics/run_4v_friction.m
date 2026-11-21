function results=run_4v_friction(sampling_s,outFolder)
%RUN_4V_FRICTION Full five-joint RNE on the unchanged saved six-second motion.
% Default grids: 0.01 and 0.005 s. Optional grids must be among 0.02/0.01/0.005.
% Generated MAT/JSON/CSV/PNG files go to results/rne unless outFolder is given.
% AI assistance with drafting and debugging the saved-motion RNE workflow,
% including load/event handling and torque summaries.
here=fileparts(mfilename('fullpath'));
if nargin<1 || isempty(sampling_s),sampling_s=[0.01 0.005];end
if nargin<2 || isempty(outFolder),outFolder=fullfile(here,'results','rne');end
if ~isfolder(outFolder),mkdir(outFolder);end
oldPath=path;restore=onCleanup(@()path(oldPath)); %#ok<NASGU>
env=setup_shoulder_friction();
saved=load(fullfile(here,'data','trajectory-input.mat'),'traces');
available=cellfun(@(tr)tr.scenario.dt_s,saved.traces);
sampling_s=sampling_s(:)';
assert(numel(unique(sampling_s))==numel(sampling_s),'Choose each grid once.');
indices=zeros(size(sampling_s));
for g=1:numel(sampling_s)
    found=find(abs(available-sampling_s(g))<1e-12);
    assert(isscalar(found),'Available saved grids are 0.02, 0.01 and 0.005 s.');
    indices(g)=found;
end
traces=saved.traces(indices);
[original,~]=createRobot();data=ele208_source_data();
assert(strcmp(data.modelId,'ELE208-hollow-modular-2026-11-04-v2'));
assert(strcmp(data.dynamicsVariantId,'ELE208-v2-shoulder-viscous-2026-11-09-v1'));
names={'empty','nominal','maximum'};robots=struct();meta=struct();
for c=names
    name=c{1};[robots.(name),meta.(name)]=ele208_dynamics_model(name,original);
    assert(meta.(name).gripperMass==2 && ~robots.(name).fast);
    assert(isequal(robots.(name).qlim,original.qlim));
end
assert(meta.nominal.objectMass==0.10 && meta.maximum.objectMass==5);
B=data.actuator.B.*data.actuator.G.^2;
assert(norm(B-[0 11.961697 0 0 0],inf)<1e-13);
runs=cell(size(traces));
results=struct('status','PASS','modelId',data.modelId, ...
    'dynamicsVariantId',data.dynamicsVariantId,'parameterSHA256',data.parameterSHA256, ...
    'sampling_s',sampling_s,'model',meta,'environment',env, ...
    'Bjoint_Nm_s_rad',B,'RMSinterval_s',[0 6], ...
    'RMSdefinition','sqrt(sum(piecewise trapz(t,tau.^2))/6)', ...
    'motionSource','data/trajectory-input.mat; unchanged saved one-sided pieces', ...
    'constantDisturbanceAdded',false, ...
    'interpretation','Predicted demands for prescribed motion; no controller or hardware performance test');
for g=1:numel(traces)
    tr=traces{g};
    assert(tr.t(1)==0 && abs(tr.t(end)-6)<1e-12 && all(diff(tr.t)>0));
    assert(numel(tr.pieces)==12 && tr.scenario.pickup_s==1.8 && tr.scenario.release_s==4.2);
    assert(isequal(tr.scenario.P.qHome,[0 pi/4 -pi/4 0 0]));
    assert(isequal(logical(tr.loaded),tr.t>=1.8-1e-12 & tr.t<4.2-1e-12));
    runs{g}=inverseDynamics(tr,robots);
    for c={'nominal','maximum'}
        name=c{1};
        results.sampling.(name)(g)=runs{g}.(name).summary;
    end
    results.eventGrids{g}=eventRows(tr,runs{g});
    fprintf('RNE dt=%.3f s: shoulder nominal/maximum peak %.9f / %.9f N m\n', ...
        sampling_s(g),runs{g}.nominal.summary.peakAbs_Nm(2),runs{g}.maximum.summary.peakAbs_Nm(2));
end
primary=find(abs(sampling_s-0.01)<1e-12,1);
if isempty(primary),primary=1;end
tr=traces{primary};run=runs{primary};
results.primarySampling_s=sampling_s(primary);results.scenario=tr.scenario;
results.events=results.eventGrids{primary};
for c={'nominal','maximum'}
    name=c{1};a=run.(name).summary;results.summary.(name)=a;
    writetable(struct2table(summaryRows(name,a)),fullfile(outFolder,['torque-summary-' name '.csv']));
    headers=[{'t_s','carrying'},labels('q','rad'),labels('qd','rad_s'),labels('qdd','rad_s2'),labels('tau','Nm')];
    writetable(array2table([tr.t tr.loaded tr.q tr.qd tr.qdd run.(name).tau], ...
        'VariableNames',headers),fullfile(outFolder,['trajectory-' name '.csv']));
end
writetable(struct2table(results.events),fullfile(outFolder,'torque-event-limits.csv'));
save(fullfile(outFolder,'results.mat'),'results','tr','run','traces','runs');
fid=fopen(fullfile(outFolder,'results.json'),'w');assert(fid>=0);
finish=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(results,PrettyPrint=true));
plot_4v_friction(outFolder,tr,run);
end

function run=inverseDynamics(tr,robots)
reference=[130 412 200 85 24];
for c={'nominal','maximum'}
    name=c{1};out=struct('tau',[],'pieces',struct([]));energy=zeros(1,5);allT=[];allQd=[];allTau=[];
    for k=1:numel(tr.pieces)
        s=tr.pieces(k);robot=robots.empty;if s.loaded,robot=robots.(name);end
        % rne returns joint-side applied torques. No external wrench/payload.
        tau=robot.rne(s.q,s.qd,s.qdd);
        assert(all(isfinite(tau),'all'));
        energy=energy+trapz(s.t,tau.^2,1);
        out.pieces(k).tau=tau;
        ix=1:numel(s.t);if k<numel(tr.pieces),ix=ix(1:end-1);end
        out.tau=[out.tau;tau(ix,:)]; %#ok<AGROW>
        allT=[allT;s.t];allQd=[allQd;s.qd];allTau=[allTau;tau]; %#ok<AGROW>
    end
    [peak,ix]=max(abs(allTau),[],1);linear=sub2ind(size(allTau),ix,1:5);
    [maxspeed,is]=max(abs(allQd),[],1);ls=sub2ind(size(allTau),is,1:5);
    summ=struct('peakAbs_Nm',peak,'rms_Nm',sqrt(energy/6), ...
        'signedPeak_Nm',allTau(linear),'peakTime_s',allT(ix)', ...
        'speedAtTorquePeak_rpm',abs(allQd(linear))*60/(2*pi), ...
        'maxAbsSpeed_rpm',maxspeed*60/(2*pi),'torqueAtMaxSpeed_Nm',allTau(ls), ...
        'meanAbsSpeed_rpm',trapz(tr.t,abs(tr.qd),1)/6*60/(2*pi));
    for j=1:5
        [summ.timeAbove90pctPeak_s(j),summ.longestAbove90pctPeak_s(j)]=aboveTime(allT,allTau(:,j),0.9*peak(j));
        [summ.timeAboveReference_s(j),summ.longestAboveReference_s(j)]=aboveTime(allT,allTau(:,j),reference(j));
    end
    out.summary=summ;run.(name)=out;
end
end

function [total,longest]=aboveTime(t,y,level)
% Integrate threshold durations with linear interpolation between samples.
total=0;longest=0;current=0;
for k=1:numel(t)-1
    dt=t(k+1)-t(k);a=abs(y(k));b=abs(y(k+1));
    if dt==0
        if b<level,current=0;end
        continue;
    end
    if a>=level && b>=level,d=dt;
    elseif a<level && b<level,d=0;
    elseif a>=level,d=dt*(a-level)/(a-b);
    else,d=dt*(b-level)/(b-a);end
    if a<level,current=0;end
    current=current+d;total=total+d;longest=max(longest,current);
    if b<level,current=0;end
end
end

function r=summaryRows(name,s)
for j=1:5
    r(j)=struct('loadCase',name,'joint',j,'peakAbs_Nm',s.peakAbs_Nm(j), ...
        'rms_0to6s_Nm',s.rms_Nm(j),'signedPeak_Nm',s.signedPeak_Nm(j), ...
        'peakTime_s',s.peakTime_s(j),'speedAtTorquePeak_rpm',s.speedAtTorquePeak_rpm(j), ...
        'maxAbsSpeed_rpm',s.maxAbsSpeed_rpm(j),'torqueAtMaxSpeed_Nm',s.torqueAtMaxSpeed_Nm(j), ...
        'meanAbsSpeed_rpm',s.meanAbsSpeed_rpm(j), ...
        'longestAbove90pctPeak_s',s.longestAbove90pctPeak_s(j), ...
        'timeAboveReference_s',s.timeAboveReference_s(j)); %#ok<AGROW>
end
end
function names=labels(prefix,unit)
names=arrayfun(@(j)sprintf('%s%d_%s',prefix,j,unit),1:5,'UniformOutput',false);
end
function rows=eventRows(tr,run)
rows=struct([]);
for c={'nominal','maximum'}
    name=c{1};
    for k=1:numel(tr.pieces)-1
        a=tr.pieces(k);b=tr.pieces(k+1);
        for j=1:5
            x=run.(name).pieces(k).tau(end,j);y=run.(name).pieces(k+1).tau(1,j);
            entry=struct('loadCase',name,'time_s',a.t(end),'joint',j, ...
                'before',a.name,'after',b.name,'loadedBefore',a.loaded,'loadedAfter',b.loaded, ...
                'qddBefore_rad_s2',a.qdd(end,j),'qddAfter_rad_s2',b.qdd(1,j), ...
                'tauBefore_Nm',x,'tauAfter_Nm',y,'jump_Nm',y-x);
            if isempty(rows),rows=entry;else,rows(end+1)=entry;end %#ok<AGROW>
        end
    end
end
end
