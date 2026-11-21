function demands=actuator_demands_4v(outFolder)
%ACTUATOR_DEMANDS_4V Separate ideal motor-equivalent and reducer output loads.
% Postprocess saved RNE. No trajectory or dynamics is rerun.
% AI assistance with drafting the motor/reducer torque postprocessing
% and checking the rotor-inertia and viscous-friction bookkeeping.
if nargin<1 || isempty(outFolder)
    outFolder=fullfile(fileparts(mfilename('fullpath')),'results','rne');
end
here=outFolder;s=load(fullfile(here,'results.mat'));
assert(strcmp(s.results.status,'PASS'));
G=121;Jrotor=5.051145;B=11.961697;
demands=struct('status','PASS','executedUTC',char(datetime('now','TimeZone','UTC', ...
    'Format','yyyy-MM-dd''T''HH:mm:ssXXX')),'ratioMagnitude',G, ...
    'interpretation','Ideal model quantities; no measured losses or achieved performance', ...
    'motorEquivalent','tauRNE2/G; includes rotor acceleration and assumed motor viscous friction', ...
    'reducerOutput','tauRNE2 - G^2*Jm*qdd2 - Bjoint*qd2; rigid-body demand, gear inertia/losses omitted');
rows=struct([]);traces=cell(size(s.traces));
for g=1:numel(s.traces)
    tr=s.traces{g};out=struct();
    for c={'nominal','maximum'}
        name=c{1};energy=0;peak=0;peakTime=0;peakSpeed=0;maxSpeed=0;torqueAtMaxSpeed=0;parts=cell(1,numel(tr.pieces));
        for p=1:numel(tr.pieces)
            a=tr.pieces(p);tau=s.runs{g}.(name).pieces(p).tau(:,2);
            transmitted=tau-Jrotor*a.qdd(:,2)-B*a.qd(:,2);parts{p}=transmitted;
            energy=energy+trapz(a.t,transmitted.^2);
            [value,i]=max(abs(transmitted));
            if value>peak,peak=value;peakTime=a.t(i);peakSpeed=abs(a.qd(i,2))*60/(2*pi);end
            [speed,i]=max(abs(a.qd(:,2))*60/(2*pi));
            if speed>maxSpeed,maxSpeed=speed;torqueAtMaxSpeed=transmitted(i);end
        end
        motor=s.runs{g}.(name).summary;
        r=struct('loadCase',name,'sampling_s',s.results.sampling_s(g), ...
            'reducerOutputPeak_Nm',peak,'reducerOutputRMS_Nm',sqrt(energy/6), ...
            'reducerOutputPeakTime_s',peakTime,'outputSpeedAtTorquePeak_rpm',peakSpeed, ...
            'maximumOutputSpeed_rpm',maxSpeed,'reducerOutputTorqueAtMaximumSpeed_Nm',torqueAtMaxSpeed, ...
            'motorEquivalentPeak_Nm',motor.peakAbs_Nm(2)/G, ...
            'motorEquivalentRMS_Nm',motor.rms_Nm(2)/G, ...
            'motorSpeedAtEquivalentTorquePeak_rpm',motor.speedAtTorquePeak_rpm(2)*G, ...
            'maximumMotorSpeed_rpm',maxSpeed*G);
        if isempty(rows),rows=r;else,rows(end+1)=r;end %#ok<AGROW>
        out.(name)=parts;
    end
    traces{g}=out;
end
demands.sampling=rows;
writetable(struct2table(rows),fullfile(here,'shoulder-rating-demands.csv'));
fid=fopen(fullfile(here,'actuator-demands.json'),'w');assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(demands,PrettyPrint=true));fclose(fid);
save(fullfile(here,'actuator-demands.mat'),'demands','traces');
disp(struct2table(rows));
end
