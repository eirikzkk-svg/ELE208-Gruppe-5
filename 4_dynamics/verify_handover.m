function verification=verify_handover()
%VERIFY_HANDOVER Reproduce current outputs and compare packaged saved evidence.
% Configure the external toolboxes with setup_shoulder_friction first.
% Writes only results/. This is a packaging check, not a new model test suite.
here=fileparts(mfilename('fullpath'));
oldPath=path;restore=onCleanup(@()path(oldPath)); %#ok<NASGU>
env=setup_shoulder_friction();
expected=jsondecode(fileread(fullfile(here,'data','expected-results.json')));
verification=struct('status','RUNNING','matlabVersion',version, ...
    'release',version('-release'),'architecture',computer, ...
    'modelId',expected.modelId,'dynamicsVariantId',expected.dynamicsVariantId, ...
    'executedUTC',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')), ...
    'externalToolboxes',jsondecode(fileread(fullfile(here,'external-toolboxes.json'))), ...
    'packagePathsConfirmed',false,'maximumCoefficientDifference',0, ...
    'maximumRNESummaryDifference',0,'maximumRNEEventDifference_Nm',0, ...
    'maximumShoulderStateDifference',0,'maximumShoulderErrorMetricDifference',0);
for name={'run_4v_friction','validate_shoulder_friction','ele208_dynamics_model', ...
        'ele208_source_data','ele208_joint2_coefficients','setup_shoulder_friction'}
    assert(strcmp(which(name{1}),fullfile(here,[name{1} '.m'])));
end
assert(strcmp(env.createRobot,fullfile(here,'support','createRobot.m')));
verification.packagePathsConfirmed=true;
for loadCase={'empty','maximum'}
    name=loadCase{1};demo=example_current_model(name);p=demo.p;
    saved=expected.shoulder.savedModelChecks.(name);
    actual=[demo.M_kgm2(2,2),demo.holdingTorque_Nm(2),p.Kg_Nm_rad, ...
        p.B_Nm_s_rad,p.JrotorAlreadyIncluded_kgm2,demo.info.movingMass];
    target=[saved.J_kgm2,saved.tau0_Nm,saved.Kg_Nm_rad,saved.B_Nm_s_rad, ...
        saved.rotorAlreadyIncluded_kgm2,saved.movingMass_kg];
    discrepancy=max(abs(actual-target));assert(discrepancy<1e-10);
    verification.maximumCoefficientDifference=max(verification.maximumCoefficientDifference,discrepancy);
    verification.shoulderCoefficients.(name)=struct('J_kgm2',actual(1), ...
        'tau0_Nm',actual(2),'Kg_Nm_rad',actual(3),'B_Nm_s_rad',actual(4));
end
rne=run_4v_friction([0.01 0.005]);
for g=1:numel(rne.sampling_s)
    savedIndex=find(abs(expected.rne.sampling_s-rne.sampling_s(g))<1e-12);
    assert(isscalar(savedIndex));
    for loadCase={'nominal','maximum'}
        name=loadCase{1};actual=rne.sampling.(name)(g);
        target=expected.rne.sampling.(name)(savedIndex);
        for field={'peakAbs_Nm','rms_Nm','signedPeak_Nm','peakTime_s','speedAtTorquePeak_rpm'}
            difference=abs(actual.(field{1})(:)-target.(field{1})(:));
            discrepancy=max(difference);assert(discrepancy<1e-9);
            verification.maximumRNESummaryDifference=max(verification.maximumRNESummaryDifference,discrepancy);
        end
    end
end
assert(numel(rne.events)==110 && numel(expected.rne.events)==110);
for k=1:numel(rne.events)
    actual=rne.events(k);target=expected.rne.events(k);
    assert(strcmp(actual.loadCase,target.loadCase) && actual.joint==target.joint);
    assert(abs(actual.time_s-target.time_s)<1e-12);
    difference=max(abs([actual.tauBefore_Nm-target.tauBefore_Nm, ...
        actual.tauAfter_Nm-target.tauAfter_Nm,actual.jump_Nm-target.jump_Nm]));
    assert(difference<1e-9);
    verification.maximumRNEEventDifference_Nm=max(verification.maximumRNEEventDifference_Nm,difference);
end
verification.rneSampling_s=rne.sampling_s;
verification.rnePrimarySummary=rne.summary;
verification.rneFineSummary=struct('nominal',rne.sampling.nominal(2), ...
    'maximum',rne.sampling.maximum(2));
verification.rneEventRows=numel(rne.events);
demands=actuator_demands_4v();
verification.shoulderMotorAndReducerDemands=demands.sampling;
shoulder=validate_shoulder_friction();
for loadCase={'empty','maximum'}
    name=loadCase{1};
    for mode={'continuous','holding'}
        treatment=mode{1};actual=shoulder.responses.(name).(treatment);
        target=expected.shoulder.responses.(name).(treatment);
        for field={'finalNonlinearState_rad_rad_s','finalLinearState_rad_rad_s'}
            discrepancy=max(abs(actual.(field{1})(:)-target.(field{1})(:)));
            assert(discrepancy<1e-8);
            verification.maximumShoulderStateDifference=max(verification.maximumShoulderStateDifference,discrepancy);
        end
        for window={'smallWindow','fullWindow'}
            for field={'maxAngleError_rad','maxSpeedError_rad_s'}
                discrepancy=abs(actual.(window{1}).(field{1})-target.(window{1}).(field{1}));
                assert(discrepancy<1e-8);
                verification.maximumShoulderErrorMetricDifference=max( ...
                    verification.maximumShoulderErrorMetricDifference,discrepancy);
            end
        end
        verification.shoulderResponses.(name).(treatment)=struct( ...
            'finalNonlinearState_rad_rad_s',actual.finalNonlinearState_rad_rad_s, ...
            'maxAngleError_rad',actual.fullWindow.maxAngleError_rad, ...
            'maxSpeedError_rad_s',actual.fullWindow.maxSpeedError_rad_s);
    end
end
verification.scope=['One current-package reproduction: model examples, saved-motion RNE on two grids, ' ...
    'existing four shoulder comparisons and their built-in tighter-solver check. No historical model suite, ' ...
    'new trajectory/IK, controller, ROS/Gazebo or hardware test.'];
verification.status='PASS';
fid=fopen(fullfile(here,'results','verification.json'),'w');assert(fid>=0);
finish=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(verification,PrettyPrint=true));
fprintf('PASS: current package reproduces saved RNE and shoulder evidence.\n');
end
