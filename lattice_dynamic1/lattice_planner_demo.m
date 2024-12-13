function [sys,x0,str,ts] = lattice_planner_demo(t,x,u,flag)

switch flag
    case 0
        [sys,x0,str,ts] = mdlInitializeSizes; % Initialization

    case 2
        sys = mdlUpdates(t,x,u);

    case 3
        sys = mdlOutputs(t,x,u);

    case {1,4,9}
        sys = [];

    otherwise
        error(['unhandled flag = ',num2str(flag)]);
end


    function [sys,x0,str,ts] = mdlInitializeSizes
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 6;
    sizes.NumOutputs     = 5;
    sizes.NumInputs      = 46;
    sizes.DirFeedthrough = 1; % Matrix D is non-empty.
    sizes.NumSampleTimes = 1;
    sys = simsizes(sizes);

    %状态量初始化
    x0=[0;0;0;0;0;0];
    str = [];             % Set str to an empty matrix.
    ts  = [0.1 0];       % sample time: [period, offset]

    global VehiclePara; % for SUV
    VehiclePara.Lf  = 1.05;  % 1.05
    VehiclePara.Lr  = 1.55;  % 1.55
    VehiclePara.L   = 2.66;  %VehiclePara.Lf + VehiclePara.Lr;
    VehiclePara.m   = 1600;   %m为车辆质量,Kg; Sprung mass = 1370
    VehiclePara.g   = 9.81;
    VehiclePara.Iz  = 2059.2;   %I为车辆绕Z轴的转动惯量，车辆固有参数
    VehiclePara.hCG = 0.65;     %m
    VehiclePara.Ix  = 700.7;   %I为车辆绕x轴的转动惯量，车辆固有参数
    VehiclePara.Tr  = 1.565;  %c,or 1.57. 注意半轴长度lc还未确定


    global futureTrajectory  scenario refPath connector;
    % 非ego车辆未来轨迹
    futureTrajectory = repelem(struct('Trajectory',[]),1,1);
    scenario = ScenarioEnv;
    
    refPath  = scenario.refPath;
    connector = TrajectoryGeneratorFrenet(refPath);

    global  laneWidth numActors Geometry replanRate latDevWeight...
          timeHorizons maxHorizon planningConstraints speedLimit safetyGap safetyTH;


    % 参考线
    laneWidth = scenario.laneWidth;
    scenario.SampleTime = connector.TimeResolution;

    % 很重要
    numActors = 2;


    scenario.numActor=numActors;
    scenario.show();

    % 动态胶囊参数
    Geometry.Length = 4.7; % in meters
    Geometry.Radius = 1.8/2; % in meters
    Geometry.FixedTransform = -1.175; % in meters

    % 规划参数
    replanRate = 1/ts(1); % Hz
    scenario.replanRate = replanRate;

    timeHorizons = [1.5]; % in seconds
    maxHorizon = max(timeHorizons); % in seconds
    scenario.maxHorizon = maxHorizon;

    latDevWeight    =  1;
    planningConstraints. maxAcceleration =  10; % in meters/second^2
    planningConstraints. maxCurvature    =   1; % 1/meters, or radians/meter
    planningConstraints. minVelocity     =   0; % in meters/second


    speedLimit = 110/3.6; % in meters/second
    safetyGap = 10; % in meters

    %TH指标
    safetyTH =2;


    global Driving_riskObj
    Driving_riskObj=Driving_riskfield();

    function sys = mdlUpdates(t,x,u)
    sys = x;

    function sys = mdlOutputs(t,x,u)

    tic
    global futureTrajectory  scenario refPath connector;

    global laneWidth numActors Geometry ;
   
    global VehStateMeasured  ParaHAT;
    
    % 输入 [X0 Y0 Yaw dyaw/ds Vx Ax]
    global egoState 

    egoState = [u(1),u(2),u(3),u(6)/u(4),u(4),u(7)];
    [VehStateMeasured, ParaHAT] = func_StateEstimation(u);

    %%
    % 场景更新
    scenario.update(futureTrajectory);
 
    % 获取场景内Actor的信息
    [curActorState,futureTrajectory] = scenario.getActorInfo();
    
    % 轨迹终点状态采样
    [allTS, allDT, numTS] = SamplingEndcontions(refPath, laneWidth, egoState);
   
    % global转frenet
    egoFrenetState = refPath.global2frenet(egoState);

    % 轨迹生成
    [frenetTraj,sv,dv,globalTraj] = connector.connect(egoFrenetState,allTS,allDT);

    % 碰撞检测--设置actor的未来轨迹
    for i = 1:numActors
        actorPoses(i).States = [futureTrajectory(i).Trajectory(:,1:3) futureTrajectory(i).Trajectory(:,5)];
    end

    % 轨迹评估
    [costTS]= EvaluateTSCost(egoFrenetState,allTS,allDT,globalTraj, frenetTraj,actorPoses);

  
   
     % 轨迹筛选
    isValid = EvaluateTrajectory(globalTraj);
    % 按照代价对轨迹进行排序
    [cost, idx] = sort(costTS);

   % 开始轨迹
    optimalTrajectory = [];

    for i = 1:numel(idx)
        % 根据筛选的结果选取有效的轨迹
        if isValid(idx(i))
            % 设置ego的未来轨迹
            egoPoses.States = globalTraj(idx(i)).Trajectory(:,1:3);
            % 碰撞检测
            isColliding = checkTrajCollision(egoPoses, actorPoses, Geometry);
            if all(~isColliding)
                % 无碰撞，则找到最优的全局路径
                optimalTrajectory = globalTraj(idx(i)).Trajectory;
                break;
            end
        end
    end

index=idx(i);
% result=0;
result=[];


if isempty(optimalTrajectory)
    % 如果没有找到轨迹，则报错
    error('No valid trajectory has been found.');
else
    % 更新绘图
    scenario.updateShow(egoState,curActorState,globalTraj, optimalTrajectory,isValid);
end


    global optimalglobalTrajectory_X optimalglobalTrajectory_Y optimalglobalTrajectory_PHI
    
    optimalglobalTrajectory_X  = optimalTrajectory(:,1);
    optimalglobalTrajectory_Y  = optimalTrajectory(:,2);
    optimalglobalTrajectory_PHI= optimalTrajectory(:,3);

    sys(1)= index;
    sys(2)= toc;
    sys(3)= optimalTrajectory(2,1);

    if abs(optimalTrajectory(end,5)-u(4))<0.01
        sys(4)= u(4);
    else
        sys(4)= optimalTrajectory(2,5);
    end

    sys(5)= optimalTrajectory(2,6);







    

  







































