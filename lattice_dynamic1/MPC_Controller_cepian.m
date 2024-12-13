function [sys,x0,str,ts] = MPC_Controller_cepian(t,x,u,flag)
% 该程序功能：用LTV MPC 和车辆简化动力学模型（小角度假设）设计控制器，作为Simulink的控制器
% 程序版本 V1.0，MATLAB版本：R2011a,采用S函数的标准形式，
% 程序编写日期 2013.12.11
% 最近一次改写 2013.12.16
% 状态量=[y_dot,x_dot,phi,phi_dot,Y,X]，控制量为前轮偏角delta_f
switch flag,
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes; % Initialization

    case 2
        sys = mdlUpdates(t,x,u); % Update discrete states

    case 3
        sys = mdlOutputs(t,x,u); % Calculate outputs

        %  case 4
        %   sys = mdlGetTimeOfNextVarHit(t,x,u); % Get next sample time

    case {1,4,9} % Unused flags
        sys = [];

    otherwise
        error(['unhandled flag = ',num2str(flag)]); % Error handling
end
% End of dsfunc.

%==============================================================
% Initialization
%==============================================================

function [sys,x0,str,ts] = mdlInitializeSizes

% Call simsizes for a sizes structure, fill it in, and convert it
% to a sizes array.

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 6;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 7;%%%%%%%%%%%%%%%%%%%
sizes.DirFeedthrough = 1; % Matrix D is non-empty.
sizes.NumSampleTimes = 1;
sys = simsizes(sizes);
x0 =[0.001;0.0001;0.0001;0.00001;0.00001;0.00001];
global x_start;
x_start=zeros(10+1,1);

% global x;
% x = zeros(md.ne + md.pye + md.me + md.Hu*md.me,1);
% Initialize the discrete states.
str = [];             % Set str to an empty matrix.
ts  = [0.02 0];       % sample time: [period, offset]
%End of mdlInitializeSizes

%==============================================================
% Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)

sys = x;
%End of mdlUpdate.

%==============================================================
% Calculate outputs
%==============================================================
function sys = mdlOutputs(t,x,u)
tic

global VehStateMeasured  ParaHAT;
global a b;
global x_start;
%global kesi;
tic
Nx=6;%状态量的个数
Nu=1;%控制量的个数
Ny=2;%输出量的个数
Np =30;%预测步长%15
Nc=10;%控制步长%2
Row=100;%松弛因子权重
Ny_cepian = 4;
c = 1;
%输入接口转换,x_dot后面加一个非常小的数，是防止出现分母为零的情况
y_dot=u(5);
x_dot=u(4)+0.000001;
phi=u(3);
phi_dot=u(6);
Y=u(2);
X=u(1);

%% 根据规划器的输入确定参考轨迹%%%%%%%%%%%%%%%%%%%%
%从端口读入局部规划器的数据，传递进来的是4次曲线的参数
    global  optimalglobalTrajectory_X 
    global  optimalglobalTrajectory_Y
    global  optimalglobalTrajectory_PHI

%% 车辆参数输入
Sf=0.2; Sr=0.2;
%syms lf lr;%前后车轮距离车辆质心的距离，车辆固有参数
lf=1.05;lr=1.61;
%syms C_cf C_cr C_lf C_lr;%分别为前后车轮的纵横向侧偏刚度，车辆固有参数
Ccf=-VehStateMeasured.CafHat;
Clf=-VehStateMeasured.CafHat;
Ccr=-VehStateMeasured.CarHat;
Clr=-VehStateMeasured.CarHat;

% Ccf=66900;Ccr=62700;
% Clf=66900;Clr=62700;

%syms m g I;%m为车辆质量，g为重力加速度，I为车辆绕Z轴的转动惯量，车辆固有参数
m=1600;g=9.8;I=2059;

%% 参考轨迹生成

% dphi_ref=zeros(Np,1);
%%矩阵转换
%  以下计算kesi,即状态量与控制量合在一起
kesi=zeros(Nx+Nu,1);
kesi(1)=y_dot;%u(1)==X(1)
kesi(2)=x_dot;%u(2)==X(2)
kesi(3)=phi; %u(3)==X(3)
kesi(4)=phi_dot;
kesi(5)=Y;
kesi(6)=X;
kesi(7)=VehStateMeasured.fwa;
delta_f=VehStateMeasured.fwa;
U=VehStateMeasured.fwa;

T=0.02;
T_all=40;

%权重矩阵设置
Q_cell=cell(Np,Np);
for i=1:1:Np
    for j=1:1:Np
        if i==j
            %Q_cell{i,j}=[200 0;0 100;];
            Q_cell{i,j}=[2500 0;0 100];
        else
            Q_cell{i,j}=zeros(Ny,Ny);
        end
    end
end

M_cell = cell(Nc,Nc);
for i=1:1:Nc
    for j=1:1:Nc
        if j<=i
            M_cell{i,j}=1;
        else
            M_cell{i,j}=zeros(Nu,Nu);
        end
    end
end
MM = cell2mat(M_cell);

R=5*10^4*eye(Nu*Nc);

%最基本也最重要的矩阵，是控制器的基础，采用动力学模型，该矩阵与车辆参数密切相关，通过对动力学方程求解雅克比矩阵得到


     a = [1.0 - (162.0*T)/x_dot, -1.0*T*(phi_dot + (0.00125*(97812.0*phi_dot - 62700.0*y_dot))/x_dot^2 - (83.625*(1.05*phi_dot + y_dot))/x_dot^2),                                        0,       -1.0*T*(x_dot - 34.45/x_dot),   0,   0
 T*(phi_dot + (83.625*delta_f)/x_dot),                                                            1.0 - (83.625*T*delta_f*(1.05*phi_dot + y_dot))/x_dot^2,                                        0, T*(y_dot + (87.80*delta_f)/x_dot),   0,   0
                                    0,                                                                                                                   0,                                      1.0,                                  T,   0,   0
                     (27.57*T)/x_dot,             T*((70.24*(1.05*phi_dot + y_dot))/x_dot^2 + (0.0005*(305173.0*phi_dot - 195624.0*y_dot))/x_dot^2),                                        0,             1.0 - (226.34*T)/x_dot,   0,   0
                           T*cos(phi),                                                                                                          T*sin(phi),  T*(x_dot*cos(phi) - 1.0*y_dot*sin(phi)),                                  0, 1.0,   0
                     -1.0*T*sin(phi),                                                                                                          T*cos(phi), -1.0*T*(y_dot*cos(phi) + x_dot*sin(phi)),                                  0,   0, 1.0 ];

     b=[                                              
                                                         83.625*T
 -1.0*T*(167.25*delta_f - (83.625*(1.05*phi_dot + y_dot))/x_dot)
                                                                0
                                                         70.245*T   
                                                                0 
                                                                0
 ];


d_k=zeros(Nx,1);%计算偏差
state_k1=zeros(Nx,1);%预测的下一时刻状态量，用于计算偏差
%以下即为根据离散非线性模型预测下一时刻状态量
%注意，为避免前后轴距的表达式（a,b）与控制器的a,b矩阵冲突，将前后轴距的表达式改为lf和lr
state_k1(1,1)=y_dot+T*(-x_dot*phi_dot+2*(Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)+Ccr*(lr*phi_dot-y_dot)/x_dot)/m);
state_k1(2,1)=x_dot+T*(y_dot*phi_dot+2*(Clf*Sf+Clr*Sr+Ccf*delta_f*(delta_f-(y_dot+phi_dot*lf)/x_dot))/m);
state_k1(3,1)=phi+T*phi_dot;
state_k1(4,1)=phi_dot+T*((2*lf*Ccf*(delta_f-(y_dot+lf*phi_dot)/x_dot)-2*lr*Ccr*(lr*phi_dot-y_dot)/x_dot)/I);
state_k1(5,1)=Y+T*(x_dot*sin(phi)+y_dot*cos(phi));
state_k1(6,1)=X+T*(x_dot*cos(phi)-y_dot*sin(phi));

d_k=state_k1-a*kesi(1:6,1)-b*kesi(7,1);%根据falcone公式（2.11b）求得d(k,t)
d_piao_k=zeros(Nx+Nu,1);%d_k的增广形式，参考falcone(B,4c)
d_piao_k(1:6,1)=d_k;
d_piao_k(7,1)=0;




%% 侧偏角e(t)计算 这段主要用来后面的侧偏角软约束 用以实现车辆在极端工况下的安全行驶
y_dot = state_k1(1,1);  %%  y_dot  x_dot  phi_dot主要用来更新 Alpha_fl r ..等侧偏角
x_dot = state_k1(2,1);  %%  y_dot  x_dot  phi_dot主要用来更新 Alpha_fl r ..等侧偏角
phi_dot = state_k1(4,1);%%  y_dot  x_dot  phi_dot主要用来更新 Alpha_fl r ..等侧偏角


Alpha_fl = atan(( (y_dot+lf*phi_dot)*cos(U) - (x_dot-c*phi_dot)*sin(U))/( (y_dot+lf*phi_dot)*sin(U) + (x_dot-c*phi_dot)*cos(U)));
Alpha_fr = atan(( (y_dot+lf*phi_dot)*cos(U) - (x_dot+c*phi_dot)*sin(U))/( (y_dot+lf*phi_dot)*sin(U) + (x_dot+c*phi_dot)*cos(U)));
Alpha_rl = atan(( (y_dot-lr*phi_dot)*cos(U) - (x_dot-c*phi_dot)*sin(U))/( (y_dot-lr*phi_dot)*sin(U) + (x_dot-c*phi_dot)*cos(U)));
Alpha_rr = atan(( (y_dot-lr*phi_dot)*cos(U) - (x_dot+c*phi_dot)*sin(U))/( (y_dot-lr*phi_dot)*sin(U) + (x_dot+c*phi_dot)*cos(U)));

% state2_k1(1,1) = ParaHAT.alpha_l1 
% state2_k1(2,1) = ParaHAT.alpha_r1 
% state2_k1(3,1) = ParaHAT.alpha_l2 
% state2_k1(4,1) = ParaHAT.alpha_r2 


state2_k1(1,1) = Alpha_fl;
state2_k1(2,1) = Alpha_fr;
state2_k1(3,1) = Alpha_rl;
state2_k1(4,1) = Alpha_rr;

D_d = [-1 ;-1; -1 ;-1]; %% 侧偏角输出矩阵 D

C_d =  [ -(1.0*(cos(delta_f)/(cos(delta_f)*(phi_dot - 1.0*x_dot) - 1.0*sin(delta_f)*(1.05*phi_dot + y_dot)) + (sin(delta_f)*(cos(delta_f)*(1.05*phi_dot + y_dot) + sin(delta_f)*(phi_dot - 1.0*x_dot)))/(cos(delta_f)*(phi_dot - 1.0*x_dot) - 1.0*sin(delta_f)*(1.05*phi_dot + y_dot))^2))/((cos(delta_f)*(1.05*phi_dot + y_dot) + sin(delta_f)*(phi_dot - 1.0*x_dot))^2/(cos(delta_f)*(phi_dot - 1.0*x_dot) - 1.0*sin(delta_f)*(1.05*phi_dot + y_dot))^2 + 1.0),             (sin(delta_f)/(cos(delta_f)*(phi_dot - 1.0*x_dot) - 1.0*sin(delta_f)*(1.05*phi_dot + y_dot)) - (1.0*cos(delta_f)*(cos(delta_f)*(1.05*phi_dot + y_dot) + sin(delta_f)*(phi_dot - 1.0*x_dot)))/(cos(delta_f)*(phi_dot - 1.0*x_dot) - 1.0*sin(delta_f)*(1.05*phi_dot + y_dot))^2)/((cos(delta_f)*(1.05*phi_dot + y_dot) + sin(delta_f)*(phi_dot - 1.0*x_dot))^2/(cos(delta_f)*(phi_dot - 1.0*x_dot) - 1.0*sin(delta_f)*(1.05*phi_dot + y_dot))^2 + 1.0), 0,              -(1.0*((1.05*cos(delta_f) + sin(delta_f))/(cos(delta_f)*(phi_dot - 1.0*x_dot) - 1.0*sin(delta_f)*(1.05*phi_dot + y_dot)) - (1.0*(cos(delta_f)*(1.05*phi_dot + y_dot) + sin(delta_f)*(phi_dot - 1.0*x_dot))*(cos(delta_f) - 1.05*sin(delta_f)))/(cos(delta_f)*(phi_dot - 1.0*x_dot) - 1.0*sin(delta_f)*(1.05*phi_dot + y_dot))^2))/((cos(delta_f)*(1.05*phi_dot + y_dot) + sin(delta_f)*(phi_dot - 1.0*x_dot))^2/(cos(delta_f)*(phi_dot - 1.0*x_dot) - 1.0*sin(delta_f)*(1.05*phi_dot + y_dot))^2 + 1.0), 0, 0
              (cos(delta_f)/(sin(delta_f)*(1.05*phi_dot + y_dot) + cos(delta_f)*(phi_dot + x_dot)) - (sin(delta_f)*(1.0*cos(delta_f)*(1.05*phi_dot + y_dot) - sin(delta_f)*(phi_dot + x_dot)))/(sin(delta_f)*(1.05*phi_dot + y_dot) + cos(delta_f)*(phi_dot + x_dot))^2)/((1.0*cos(delta_f)*(1.05*phi_dot + y_dot) - sin(delta_f)*(phi_dot + x_dot))^2/(sin(delta_f)*(1.05*phi_dot + y_dot) + cos(delta_f)*(phi_dot + x_dot))^2 + 1.0),                              -(1.0*(sin(delta_f)/(sin(delta_f)*(1.05*phi_dot + y_dot) + cos(delta_f)*(phi_dot + x_dot)) + (1.0*cos(delta_f)*(1.0*cos(delta_f)*(1.05*phi_dot + y_dot) - sin(delta_f)*(phi_dot + x_dot)))/(sin(delta_f)*(1.05*phi_dot + y_dot) + cos(delta_f)*(phi_dot + x_dot))^2))/((1.0*cos(delta_f)*(1.05*phi_dot + y_dot) - sin(delta_f)*(phi_dot + x_dot))^2/(sin(delta_f)*(1.05*phi_dot + y_dot) + cos(delta_f)*(phi_dot + x_dot))^2 + 1.0), 0,                                             ((1.05*cos(delta_f) - 1.0*sin(delta_f))/(sin(delta_f)*(1.05*phi_dot + y_dot) + cos(delta_f)*(phi_dot + x_dot)) - ((1.0*cos(delta_f)*(1.05*phi_dot + y_dot) - sin(delta_f)*(phi_dot + x_dot))*(cos(delta_f) + 1.05*sin(delta_f)))/(sin(delta_f)*(1.05*phi_dot + y_dot) + cos(delta_f)*(phi_dot + x_dot))^2)/((1.0*cos(delta_f)*(1.05*phi_dot + y_dot) - sin(delta_f)*(phi_dot + x_dot))^2/(sin(delta_f)*(1.05*phi_dot + y_dot) + cos(delta_f)*(phi_dot + x_dot))^2 + 1.0), 0, 0
             -(1.0*(cos(delta_f)/(cos(delta_f)*(phi_dot - 1.0*x_dot) + sin(delta_f)*(1.61*phi_dot - 1.0*y_dot)) + (1.0*sin(delta_f)*(1.0*sin(delta_f)*(phi_dot - 1.0*x_dot) - cos(delta_f)*(1.61*phi_dot - 1.0*y_dot)))/(cos(delta_f)*(phi_dot - 1.0*x_dot) + sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2))/((1.0*sin(delta_f)*(phi_dot - 1.0*x_dot) - cos(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2/(cos(delta_f)*(phi_dot - 1.0*x_dot) + sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2 + 1.0), (sin(delta_f)/(cos(delta_f)*(phi_dot - 1.0*x_dot) + sin(delta_f)*(1.61*phi_dot - 1.0*y_dot)) - (cos(delta_f)*(1.0*sin(delta_f)*(phi_dot - 1.0*x_dot) - cos(delta_f)*(1.61*phi_dot - 1.0*y_dot)))/(cos(delta_f)*(phi_dot - 1.0*x_dot) + sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2)/((1.0*sin(delta_f)*(phi_dot - 1.0*x_dot) - cos(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2/(cos(delta_f)*(phi_dot - 1.0*x_dot) + sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2 + 1.0), 0, ((1.61*cos(delta_f) - 1.0*sin(delta_f))/(cos(delta_f)*(phi_dot - 1.0*x_dot) + sin(delta_f)*(1.61*phi_dot - 1.0*y_dot)) + (1.0*(1.0*sin(delta_f)*(phi_dot - 1.0*x_dot) - cos(delta_f)*(1.61*phi_dot - 1.0*y_dot))*(cos(delta_f) + 1.61*sin(delta_f)))/(cos(delta_f)*(phi_dot - 1.0*x_dot) + sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2)/((1.0*sin(delta_f)*(phi_dot - 1.0*x_dot) - cos(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2/(cos(delta_f)*(phi_dot - 1.0*x_dot) + sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2 + 1.0), 0, 0
              (1.0*(cos(delta_f)/(1.0*cos(delta_f)*(phi_dot + x_dot) - sin(delta_f)*(1.61*phi_dot - 1.0*y_dot)) + (1.0*sin(delta_f)*(sin(delta_f)*(phi_dot + x_dot) + cos(delta_f)*(1.61*phi_dot - 1.0*y_dot)))/(1.0*cos(delta_f)*(phi_dot + x_dot) - sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2))/((sin(delta_f)*(phi_dot + x_dot) + cos(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2/(1.0*cos(delta_f)*(phi_dot + x_dot) - sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2 + 1.0),                -(sin(delta_f)/(1.0*cos(delta_f)*(phi_dot + x_dot) - sin(delta_f)*(1.61*phi_dot - 1.0*y_dot)) - (cos(delta_f)*(sin(delta_f)*(phi_dot + x_dot) + cos(delta_f)*(1.61*phi_dot - 1.0*y_dot)))/(1.0*cos(delta_f)*(phi_dot + x_dot) - sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2)/((sin(delta_f)*(phi_dot + x_dot) + cos(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2/(1.0*cos(delta_f)*(phi_dot + x_dot) - sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2 + 1.0), 0,                        -((1.61*cos(delta_f) + sin(delta_f))/(1.0*cos(delta_f)*(phi_dot + x_dot) - sin(delta_f)*(1.61*phi_dot - 1.0*y_dot)) - ((sin(delta_f)*(phi_dot + x_dot) + cos(delta_f)*(1.61*phi_dot - 1.0*y_dot))*(cos(delta_f) - 1.61*sin(delta_f)))/(1.0*cos(delta_f)*(phi_dot + x_dot) - sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2)/((sin(delta_f)*(phi_dot + x_dot) + cos(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2/(1.0*cos(delta_f)*(phi_dot + x_dot) - sin(delta_f)*(1.61*phi_dot - 1.0*y_dot))^2 + 1.0), 0, 0
     ];
  

CC_d_cell = cell(1,2);
CC_d_cell{1, 1} = C_d;
CC_d_cell{1, 2} = D_d;
Cc_d = cell2mat(CC_d_cell);
side_slip_error = state2_k1 - C_d*kesi(1:6,1) - D_d*kesi(7,1);%根据上面更新过来计算新的值 理论上要计算未来每个预测时域内的差值（最好不能强行理解为误差等，此处只是为了变换才这样表示，可看论文基于模型预测控制算法里面的推导公式）
sideslip_piao_k=zeros(Ny_cepian, 1);%d_k的增广形式，参考falcone(B,4c)
sideslip_piao_k(1:4,1)=side_slip_error;

%%
A_cell=cell(2,2);
B_cell=cell(2,1);
A_cell{1,1}=a;
A_cell{1,2}=b;
A_cell{2,1}=zeros(Nu,Nx);
A_cell{2,2}=eye(Nu);
B_cell{1,1}=b;
B_cell{2,1}=eye(Nu);
A=cell2mat(A_cell);
B=cell2mat(B_cell);

C=[0 0 1 0 0 0 0;
   0 0 0 0 1 0 0;];

PSI_cell=cell(Np,1);
PSI_sideslip_cell = cell(Np,1);
THETA_cell=cell(Np,Nc);
THETA_sideslip_cell=cell(Np,Nc);
THETA_sideslip_cell2=cell(Np,Nc);
GAMMA_cell=cell(Np,Np);
GAMMA_sideslip_cell=cell(Np,Np);
PHI_cell=cell(Np,1);
PHI_sideslip_cell=cell(Np,1);
for p=1:1:Np
    PHI_cell{p,1} = d_piao_k;%理论上来说，这个是要实时更新的，但是为了简便，这里又一次近似
    PHI_sideslip_cell{p,1} = sideslip_piao_k;
    for q=1:1:Np
        if q<=p
            GAMMA_cell{p,q}=C*A^(p-q);
            GAMMA_sideslip_cell{p,q}=Cc_d*A^(p-q);
        else
            GAMMA_cell{p,q} = zeros(Ny,Nx+Nu);
            GAMMA_sideslip_cell{p,q} = zeros(Ny_cepian,Nx+Nu);
        end
    end
end
for j=1:1:Np
    PSI_cell{j,1}=C*A^j;
    PSI_sideslip_cell{j,1} = Cc_d*A^j;
    for k=1:1:Nc
        if k<=j
            THETA_cell{j,k}=C*A^(j-k)*B;
            THETA_sideslip_cell{j,k} = Cc_d*A^(j-k)*B;
        else
            THETA_cell{j,k}=zeros(Ny,Nu);
            THETA_sideslip_cell{j,k} = zeros(Ny_cepian,Nu);
        end
    end
end

for j=1:1:Np
    for k=1:1:Nc
        if k == j+1
            THETA_sideslip_cell2{j,k} = D_d;
        else
            THETA_sideslip_cell2{j,k} = zeros(Ny_cepian,Nu);
        end
    end
end

PSI=cell2mat(PSI_cell);%size(PSI)=[Ny*Np Nx+Nu]
THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]
GAMMA=cell2mat(GAMMA_cell);%大写的GAMMA
PHI=cell2mat(PHI_cell);
PHI_sideslip = cell2mat(PHI_sideslip_cell);
GAMMA_sideslip = cell2mat(GAMMA_sideslip_cell);
PSI_sideslip = cell2mat(PSI_sideslip_cell);
THETA_sideslip  = cell2mat(THETA_sideslip_cell) + cell2mat(THETA_sideslip_cell2);

Q = cell2mat(Q_cell);
S = 1000;
% H=2*(THETA'*Q*THETA+R+MM'*S*MM);
% H=(H+H')/2;
H_cell=cell(2,2);
H_cell{1,1}=2*(THETA'*Q*THETA+R+MM'*S*MM);
H_cell{1,2}=zeros(Nu*Nc,1);
H_cell{2,1}=zeros(1,Nu*Nc);
H_cell{2,2}=Row;
H=cell2mat(H_cell);
H=(H+H')/2;
Yita_ref_cell = cell(Np,1);

%%
for p=1:1:Np%%%%%%%%%%%%%%%%%
    if t+p*T>T_all
        X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%惯性坐标系下纵向速度
        X_predict(Np,1)=X+X_DOT*Np*T;
        Y_ref(p,1)=interp1(optimalglobalTrajectory_X,optimalglobalTrajectory_Y,X_predict(Np,1),"linear");%Y_ref(p,1)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
        phi_ref(p,1)=interp1(optimalglobalTrajectory_X,optimalglobalTrajectory_PHI,X_predict(Np,1),"linear");%phi_ref(p,1)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));
        Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
    else
        X_DOT=x_dot*cos(phi)-y_dot*sin(phi);%惯性坐标系下纵向速度
        X_predict(p,1)=X+X_DOT*p*T; %首先计算出未来X的位置，X(t)=X+X_dot*t
        Y_ref(p,1)=interp1(optimalglobalTrajectory_X,optimalglobalTrajectory_Y,X_predict(p,1),"linear");%Y_ref(p,1)=dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
        phi_ref(p,1)=interp1(optimalglobalTrajectory_X,optimalglobalTrajectory_PHI,X_predict(p,1),"linear");%phi_ref(p,1)=atan(dy1*(1/cosh(z1))^2*(1.2/dx1)-dy2*(1/cosh(z2))^2*(1.2/dx2));
        Yita_ref_cell{p,1}=[phi_ref(p,1);Y_ref(p,1)];
    end
end



Yita_ref=cell2mat(Yita_ref_cell);
error_1=PSI*kesi+GAMMA*PHI-Yita_ref ;%求偏差
UU = ones(Nc,1)*U(1);

% f = 2*error_1'*Q*THETA + UU'*S*MM;
f_cell=cell(1,2);
f_cell{1,1}=2*error_1'*Q*THETA + UU'*S*MM;
f_cell{1,2}=0;
f=cell2mat(f_cell);

%% 以下为约束生成区域
%控制量约束
A_t=zeros(Nc,Nc);%见falcone论文 P181
for p=1:1:Nc
    for q=1:1:Nc
        if q<=p
            A_t(p,q)=1;
        else
            A_t(p,q)=0;
        end
    end
end

A_I=kron(A_t,eye(Nu));%求克罗内克积 
Ut=kron(ones(Nc,1),U(1));
umin=-0.1744;%维数与控制变量的个数相同-0.1744
umax= 0.1744;%0.1744
delta_umin=-0.0148;%-0.0148*0.4
delta_umax= 0.0148;%0.0148*0.4
Umin=kron(ones(Nc,1),umin);
Umax=kron(ones(Nc,1),umax);


%侧偏角输出软量约束
y_cepian_max = [  0.07;  0.07;  0.07; 0.07];
y_cepian_min = [ -0.07; -0.07; -0.07;-0.07];
% y_cepian_max = [  0.0489; 0.0489; 0.0489; 0.0489];
% y_cepian_min = [ -0.0489; -0.0489; -0.0489; -0.0489];
Y_cepian_max = kron(ones(Np,1),y_cepian_max);
Y_cepian_min = kron(ones(Np,1),y_cepian_min);

%横向y和航向输出量硬约束
ycmax=[ 0.3;57];
ycmin=[-0.3;43];
Ycmax=kron(ones(Np,1),ycmax);
Ycmin=kron(ones(Np,1),ycmin);

%结合控制量约束和输出量约束和软约束
 A_cons_cell={           A_I zeros(Nu*Nc,1);                             -A_I zeros(Nu*Nc,1); 
     THETA_sideslip -1*ones(Ny_cepian*Np,1);            -THETA_sideslip ones(Ny_cepian*Np,1);
                       THETA zeros(Ny*Np,1);                          -THETA zeros(Ny*Np,1)};

 % A_cons_cell={A_I ;-A_I ; 
 %     THETA_sideslip ; -THETA_sideslip ;
 %     THETA ;-THETA };

b_cons_cell={Umax-Ut;-Umin+Ut; 
            Y_cepian_max-PSI_sideslip*kesi-GAMMA_sideslip*PHI-PHI_sideslip; -Y_cepian_min+PSI_sideslip*kesi+GAMMA_sideslip*PHI + PHI_sideslip;
            Ycmax-PSI*kesi-GAMMA*PHI;-Ycmin+PSI*kesi+GAMMA*PHI};

%%结合控制量约束和输出量约束
% A_cons_cell={A_I zeros(Nu*Nc,1);-A_I zeros(Nu*Nc,1);THETA zeros(Ny*Np,1);-THETA zeros(Ny*Np,1)};
% b_cons_cell={Umax-Ut;-Umin+Ut;Ycmax-PSI*kesi-GAMMA*PHI;-Ycmin+PSI*kesi+GAMMA*PHI};

A_cons=cell2mat(A_cons_cell);%（求解方程）状态量不等式约束增益矩阵，转换为绝对值的取值范围
b_cons=cell2mat(b_cons_cell);%（求解方程）状态量不等式约束的取值

%状态量约束
M=0*pi/180;   %软约束最大值 此处M是最大的软约束约束值，当此值太大容易车量失稳，失去加入软约束的意义，太小容易造成无解，控制丢失。
delta_Umin=kron(ones(Nc,1),delta_umin);
delta_Umax=kron(ones(Nc,1),delta_umax);
lb=[delta_Umin;0];%（求解方程）状态量下界，包含控制时域内控制增量和松弛因子
ub=[delta_Umax;M];%（求解方程）状态量上界，包含控制时域内控制增量和松弛因子

%% 开始求解过程
options = optimset('Display','off', 'Algorithm', 'active-set');

[X,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,x_start,options);

% y_zmp1=PSI_sideslip*kesi+GAMMA_sideslip*PHI+THETA_sideslip*X(1:Nu*Nc,1)+PHI_sideslip;
% y_hsh=reshape(y_zmp1,4,Np)

%% 计算输出
if isempty(X)
    sys(1)=kesi(7,1);
    x_start=zeros(Nc+1,1);
else
    x_start=X;
    u_piao=X(1);%得到控制增量

    sys(1)=kesi(7,1)+u_piao;

end



