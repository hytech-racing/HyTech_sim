%Vehicle Parameters (all units SI)

Parameters.g = 9.81; %acceleration due to gravity

Parameters.curbMass = 195; %[kg]

Parameters.driverMass = 68; %[kg]

Parameters.mass = Parameters.curbMass + Parameters.driverMass; %[kg]

Parameters.L = 1.53; %wheelbase

Parameters.b = 0.78; %distance from CG to rear axle 51% front

Parameters.TmRear = 42; %motor torque
Parameters.TmFront = 42; %motor torque

Parameters.derateSpeedRatioRear = 0.75; % derate begins after derateRatio*maxMotorRPM
Parameters.derateSpeedRatioFront = 0.75; % derate begins after derateRatio*maxMotorRPM

% To update derating strategy, write new derating function and change @(functionDerate)
derateFunFront = @linearDerate; % define function handle
derateFunRear = @linearDerate; % define function handle

Parameters.derateRear = derateFunRear; % define function handle used to derate Rear motor torque
Parameters.derateFront = derateFunFront; % define function handle used to derate Front motor torque

Parameters.mRearTopSpeed = 20000; % Rear Torque drops to 0 when motor rpm exceeds this value
Parameters.mFrontTopSpeed = 20000; % Rear Torque drops to 0 when motor rpm exceeds this value

Parameters.nRear = 11.68; %gear ratio
Parameters.nFront = 11.68; %gear ratio

Parameters.r = 0.200; %tire radius
Parameters.rho = 1.225; %air density

% HIGH DRAG CONFIGURATION
Parameters.Cd = 1.23; %drag coefficient
Parameters.Cl = 3.71; % Lift coefficient

Parameters.A = 1; %frontal (reference) area

Parameters.hcp = 0; % CP height
Parameters.bcp = 0.8109; % Distance from CP to rear axle

Parameters.Crr = 0.028; %rolling resistance coefficient R25B 

% Parameters.Kf = 556.4 * 57.3; %Front roll stiffness 
% Parameters.Kr = 693 * 57.3; %Rear roll stiffness 
Parameters.Kf = 31879.813; %Front roll stiffness 
Parameters.Kr = 39701.297; %Rear roll stiffness 
Parameters.hf = 0.083699; %Front roll center height
Parameters.hr = 0.064906; %Rear roll center height 
Parameters.hg = 0.20; %CG height
Parameters.hl = Parameters.hg - (((Parameters.hr-Parameters.hf)/Parameters.L)*(Parameters.L-Parameters.b) + Parameters.hf); %Distance from CG to roll axis 

Parameters.t = 1.2; %track
Parameters.driverFactorLong = 1; % driver is not as good as sim
Parameters.driverFactorLat = 1; % driver is not as good as sim
Parameters.tireFactor = 0.56; % tires are not as good as TTC data
Parameters.yawInertia = 68; % kg*m^2;

Parameters.toeRI = 1;
Parameters.toeRO = 1;
Parameters.toeFI = 0.5;
Parameters.toeFO = 0.5;

% Accepts pressure values of 8,10,12,14 (PSI)
Parameters.pRI = 8;
Parameters.pRO = 8;
Parameters.pFI = 8;
Parameters.pFO = 8;

% Static camber values
Parameters.staticCamber = [-2,-2,-1,-1];
Parameters.tireType = 'Hoosier16x75x10LCO_7IN';
Parameters.camberFitFile = 'HT07CamberVsRollData.xlsx';
Parameters.steerCamberFitFile = 'HT07CamberVsSteerData.xlsx';

% Steer torque data
Parameters.steerTorqueFile = 'HT07LeftHandCornerSteerTorqueParams.xlsx';

% Control strategy string
Parameters.controller = "PID";
% Parameters.controller = "Normal Force";
% Parameters.controller = "Equal Torque";
