    %%
    carLen   = 4.7;
    carWidth = 1.8;
    carRear = 1.175;
    % 参考线

    scenario = ScenarioEnv;
    hold on;
    scenario.show; 

    % 参考线
    laneWidth = scenario.laneWidth;
    refPath  = scenario.refPath;
    connector = TrajectoryGeneratorFrenet(refPath);
    scenario.SampleTime = connector.TimeResolution;

    % 初始化
    numActors = 2;
    % 非ego车辆当前位姿
    actorPoses = repelem(struct('States',[]),numActors,1);
    % 非ego车辆未来轨迹
    futureTrajectory = repelem(struct('Trajectory',[]),numActors,1);

    % 动态胶囊参数
    Geometry.Length = carLen; % in meters
    Geometry.Radius = carWidth/2; % in meters
    Geometry.FixedTransform = -carRear; % in meters

    % 规划参数
    replanRate = 10; % Hz
    scenario.replanRate = replanRate;
    timeHorizons = 1:3; % in seconds
    maxHorizon = max(timeHorizons); % in seconds
    scenario.maxHorizon = maxHorizon;
    latDevWeight    =  1;
    timeWeight      = -1;
    speedWeight     =  1;
    maxAcceleration =  15; % in meters/second^2
    maxCurvature    =   1; % 1/meters, or radians/meter
    minVelocity     =   0; % in meters/second
    speedLimit = 11; % in meters/second
    safetyGap = 10; % in meters