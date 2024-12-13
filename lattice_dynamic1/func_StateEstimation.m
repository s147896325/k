function [VehStatemeasured, HATParameter] = func_StateEstimation(ModelInput)
ModelInput=ModelInput(8:end);
g = 9.81;
VehStatemeasured.X       = round(100*ModelInput(1))/100;%单位为m, 保留2位小数
VehStatemeasured.Y       = round(100*ModelInput(2))/100;%单位为m, 保留2位小数
VehStatemeasured.phi     = ModelInput(3)*pi/180; %航向角，Unit：deg-->rad，保留1位小数
VehStatemeasured.x_dot   = ModelInput(4)/3.6; %Unit:km/h-->m/s，保留1位小数
VehStatemeasured.y_dot   = ModelInput(5)/3.6; %Unit:km/h-->m/s，保留1位小数
VehStatemeasured.phi_dot = ModelInput(6)*pi/180; %Unit：deg/s-->rad/s，保留1位小数
VehStatemeasured.beta    = ModelInput(7)*pi/180;% side slip, Unit:deg-->rad，保留1位小数
%VehStatemeasured.delta_f = (round(10*(ModelInput(8)))/10)*pi/180; % deg-->rad
VehStatemeasured.delta_f = ((ModelInput(8)+ ModelInput(9))/2)*pi/180; % deg-->rad
VehStatemeasured.fwa     = VehStatemeasured.delta_f;
VehStatemeasured.Steer_SW= ModelInput(10); %deg
VehStatemeasured.Ax      = g*ModelInput(11);%单位为m/s^2, 保留2位小数
VehStatemeasured.Ay      = g*ModelInput(12);%单位为m/s^2, 保留2位小数
VehStatemeasured.yawrate_dot = ModelInput(13); %rad/s^2
% Here I don't explore the state estimation process, and deem the
% measured values are accurate!!!
HATParameter.alpha_l1   = (round(10*ModelInput(14))/10)*pi/180; % deg-->rad，保留1位小数
HATParameter.alpha_l2   = (round(10*ModelInput(15))/10)*pi/180;% deg-->rad，保留1位小数
HATParameter.alpha_r1   = (round(10*ModelInput(16))/10)*pi/180; % deg-->rad，保留1位小数
HATParameter.alpha_r2   = (round(10*ModelInput(17))/10)*pi/180; % deg-->rad，保留1位小数
HATParameter.alphaf     = (round(10*0.5 * (ModelInput(14)+ ModelInput(16)))/10)*pi/180; % deg-->rad，保留1位小数
HATParameter.alphar     = (round(10*0.5 * (ModelInput(15)+ ModelInput(17)))/10)*pi/180; % deg-->rad，保留1位小数
HATParameter.Fz_l1      = round(10*ModelInput(18))/10; % N 
HATParameter.Fz_l2      = round(10*ModelInput(19))/10; % N 
HATParameter.Fz_r1      = round(10*ModelInput(20))/10; % N 
HATParameter.Fz_r2      = round(10*ModelInput(21))/10; % N 
HATParameter.Fy_l1      = round(10*ModelInput(22))/10; % N 
HATParameter.Fy_l2      = round(10*ModelInput(23))/10; % N 
HATParameter.Fy_r1      = round(10*ModelInput(24))/10; % N 
HATParameter.Fy_r2      = round(10*ModelInput(25))/10; % N 
HATParameter.Fyf        = HATParameter.Fy_l1 + HATParameter.Fy_r1;
HATParameter.Fyr        = HATParameter.Fy_l2 + HATParameter.Fy_r2;
%HATParameter.GearStat    = ModelInput(30);
VehStatemeasured.Roll_Shad   = ModelInput(30)*pi/180;% deg-->rad
HATParameter.Roll            = ModelInput(31)*pi/180;% deg-->rad
HATParameter.Rollrate        = ModelInput(32)*pi/180;% deg/s-->rad/s
HATParameter.Roll_accel      = ModelInput(33); % rad/s^2
HATParameter.Z0              = ModelInput(34); %m
VehStatemeasured.Station     = ModelInput(35); %m
HATParameter.Zcg_TM          = ModelInput(36); %m
HATParameter.Zcg_SM          = ModelInput(37); %m
HATParameter.Ay_CG           = ModelInput(38)*g; %m/s^2

%Arfa_f = (VehStatemeasured.beta + VehStatemeasured.phi_dot*1.05/VehStatemeasured.x_dot - VehStatemeasured.fwa);



 %[yf, Calpha_f1] = func_RLSFilter_Calpha_f(Arfa_f, HATParameter.Fyf);




    
CafHat =(HATParameter.Fy_l1+HATParameter.Fy_l2)/(HATParameter.alpha_l1+HATParameter.alpha_l2);
 if CafHat > -30000 || CafHat==inf ||CafHat==-inf
    CafHat = -110000;
 end

VehStatemeasured.CafHat=CafHat;

%Arfa_r = (VehStatemeasured.beta - VehStatemeasured.phi_dot*1.55/VehStatemeasured.x_dot);

CarHat =(HATParameter.Fy_r1+HATParameter.Fy_r2)/(HATParameter.alpha_r1+HATParameter.alpha_r2);
 %[yr, Calpha_r1] = func_RLSFilter_Calpha_r(Arfa_r, HATParameter.Fyr);
 %CarHat = sum(Calpha_r1);
 if CarHat > -30000|| CarHat==inf ||CarHat==-inf
    CarHat = -92000;
 end
 VehStatemeasured.CarHat=CarHat;
