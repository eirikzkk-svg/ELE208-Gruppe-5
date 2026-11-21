function data = ele208_source_data()
%ELE208_SOURCE_DATA Masses, centres of mass and inertias for the robot.
% Units: kg, m and kg*m^2. Inertias are about each link's centre of mass.
% All properties use the local DH frames D1 to D5.
% data.robot.mass lists links 1-5 without the gripper or object.
% AI assistance with discussing mass assumptions, checking mass properties,
% and drafting and adapting this parameter-data file.
data.modelId='ELE208-hollow-modular-2026-11-04-v2';
data.dynamicsVariantId='ELE208-v2-shoulder-viscous-2026-11-09-v1';
% parameterSHA256 identifies the unchanged v2 rigid-body parameter source.
data.parameterSHA256='b26763543d1974f28a67de45bd01c55218ffe1ce73519290a9783dcbf32454e4';
data.units='kg, m, kg m^2 about each body COM, rad, N m';
data.DH=[0.29999999999999999 0 1.5707963267948966; 0.20000000000000001 0.67259999999999998 0; -0.20000000000000001 0.67259999999999998 0; 0.20000000000000001 0 1.5707963267948966; 0.20000000000000001 0 0]; % columns d,a,alpha
data.jointLimits=[-3.1415926535897931 3.1415926535897931; -1.5707963267948966 1.5707963267948966; -2.6179938779914944 0.26179938779914941; -3.1415926535897931 3.1415926535897931; -3.1415926535897931 3.1415926535897931];
data.base=eye(4); data.tool=eye(4);
data.gravityPhysical=[0 0 -9.81];
data.gravityRTB=[0 0 9.81]; % RTB holding/compensation convention
data.robot.mass=[17.562153819457354 11.032195151471099 6.8326045310738976 3.9024889024961182 1.0858479811238333];
data.robot.com=[0 -0.040270829907755698 -0.0717024046257176; -0.22939328542362977 0.0010197426573350507 -0.068913149055855147; -0.23053545537670064 0.0010244995108621914 0.014143548557186417; 0 -0.06167594831180756 -0.018382708575267841; 0 0 -0.12192669032913082];
data.robot.inertia(:,:,1)=[0.28941040625683279 0 0; 0 0.15414943350657614 0.051372188568702286; 0 0.051372188568702286 0.1965311783407627];
data.robot.inertia(:,:,2)=[0.073111243686667285 0.0012027005389841651 -0.1091905779991451; 0.0012027005389841651 0.99805554784800043 -0.00077527292687837052; -0.1091905779991451 -0.00077527292687837052 0.96198002426689644];
data.robot.inertia(:,:,3)=[0.072625380871079295 0.00074035181236309562 0.16074668182405799; 0.00074035181236309562 0.64619606765834292 9.9004839900304961e-05; 0.16074668182405799 9.9004839900304961e-05 0.59078997938834754];
data.robot.inertia(:,:,4)=[0.033293213058070842 0 0; 0 0.0079735906328678285 0.0047845286827169426; 0 0.0047845286827169426 0.033051459043399435];
data.robot.inertia(:,:,5)=[0.0074636387708573674 0 0; 0 0.0074636387708573674 0; 0 0 0.0011264302931923663];
data.fixedBase.mass=14.422254372959818;
data.fixedBase.com=[0 0 -0.10575646899461945];
data.fixedBase.inertia=[0.10980008318088294 0 0; 0 0.10980008318088294 0; 0 0 0.15310745785750907];
data.fixedBase.frame='root'; % anchored, NOT an extra moving Link
data.gripper.mass=2; data.gripper.com=[0 0 -0.06];
data.gripper.dimensions=[0.18 0.12 0.12];
data.cases.empty.objectMass=0;
data.cases.empty.objectCOM=[0 0 0.050000000000000003];
data.cases.empty.objectDimensions=[0.25 0.14999999999999999 0.10000000000000001];
data.cases.empty.body5.mass=3.0858479811238335;
data.cases.empty.body5.com=[0 0 -0.081790759649501055];
data.cases.empty.body5.inertia=[0.014962498020559711 0 0; 0 0.017962498020559711 0; 0 0 0.0089264302931923653];
data.cases.empty.movingMass=42.415290385622299;
data.cases.nominal.objectMass=0.10000000000000001;
data.cases.nominal.objectCOM=[0 0 0.0050000000000000001];
data.cases.nominal.objectDimensions=[0.16 0.080000000000000002 0.01];
data.cases.nominal.body5.mass=3.1858479811238336;
data.cases.nominal.body5.com=[0 0 -0.079066500357666145];
data.cases.nominal.body5.inertia=[0.015746284229938068 0 0; 0 0.018906284229938061 0; 0 0 0.0091930969598590321];
data.cases.nominal.movingMass=42.515290385622301;
data.cases.maximum.objectMass=5;
data.cases.maximum.objectCOM=[0 0 0.050000000000000003];
data.cases.maximum.objectDimensions=[0.25 0.14999999999999999 0.10000000000000001];
data.cases.maximum.body5.mass=8.085847981123834;
data.cases.maximum.body5.com=[0 0 -0.00029605435874949855];
data.cases.maximum.body5.inertia=[0.061646940743787355 0 0; 0 0.081313607410454028 0; 0 0 0.044343096959859031];
data.cases.maximum.movingMass=47.415290385622299;
% Include the shoulder motor's rotational inertia through G^2*Jm.
% Rotational inertia from the other drives is left out.
data.actuator.Jm=[0 3.45e-4 0 0 0]; % motor-referred kg m^2
data.actuator.G=[100 121 160 160 100]; % gear ratio magnitudes
data.actuator.reflected=data.actuator.Jm.*data.actuator.G.^2;
% RTB expects motor-side viscous B and applies G^2 internally.
% Borrowed PUMA shoulder value, assumed for simulation, not measured here.
data.actuator.B=[0 817e-6 0 0 0]; % motor-side N*m*s/rad
data.actuator.Bjoint=data.actuator.B.*data.actuator.G.^2;
data.actuator.Tc=zeros(5,2); % Coulomb friction omitted on every joint
data.friction=struct('assumed',true,'motorB_Nm_s_rad',817e-6, ...
    'jointB_Nm_s_rad',11.961697,'joint',2, ...
    'basis','Eirik 9 November 2026: borrowed PUMA 560 shoulder simulation value', ...
    'measuredForThisRobot',false,'otherJoints','viscous friction omitted', ...
    'Coulomb','omitted on all joints', ...
    'RTBSource','models/mdl_puma560.m, shoulder B=817e-6');
data.actuator.included='J2 Panasonic rotor reflection and assumed shoulder viscous friction';
data.actuator.omitted='Other rotor/gear spin; rotor-carrier coupling; other-joint viscous and all Coulomb friction; compliance; losses; brake transients';
data.actuator.shoulderSource='https://industry.panasonic.com/global/en/products/motor/fa-motor/ac-servo/number/msmf152l1h5';
% These FHA catalogue values refer to the output shaft, not the motor shaft.
% They are kept for reference and are not added to the model.
data.actuator.FHAOutputInertiaEvidence=[7.1 NaN 18.1 8.3 0.67];
end
