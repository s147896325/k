classdef ScenarioEnv < handle
    
    properties(Access=public)
        actors                  % actors信息
        numActor
        refPath                 % 参考线
        laneWidth               % 车道宽

        egop                    % ego绘图句柄
        carp                    % actors绘图句柄
        trajp                   % 轨迹绘图句柄

        trajp11
        trajopt                 % 最优轨迹绘图句柄
        cartraj                 % actor轨迹绘图句柄

        SampleTime              % 采样时间
        replanRate              % 重规划频率
        maxHorizon              % 滚动窗口长度

        fig                     % 图窗figure句柄
        ax                      % 图窗axes句柄

        futureTrajectory        % actors的未来轨迹
        curState                % actors的当前状态
    end
    methods
        function updateShow(obj, egoState, curActorState, globalTraj,optimalTrajectory,isValid)% 更新绘图
            curpos = egoState(1:3);
            xy = [-2.5 -2.5 2.5 2.5
                     1   -1  -1   1];
            M = [cos(curpos(3)), -sin(curpos(3));sin(curpos(3)) cos(curpos(3))];
            xy = M*xy+curpos(1:2)';
            set(obj.egop,'xdata',xy(1,:),'ydata',xy(2,:));

            for idx = 1:obj.numActor
                curpos = curActorState(idx,1:3);
                xy = [-2.5 -2.5 2.5 2.5
                    1   -1  -1   1];
                M = [cos(curpos(3)), -sin(curpos(3));sin(curpos(3)) cos(curpos(3))];
                xy = M*xy+curpos(1:2)';
                set(obj.cartraj(idx),'xdata', obj.futureTrajectory(idx).Trajectory(:,1),'ydata', obj.futureTrajectory(idx).Trajectory(:,2));
                set(obj.carp(idx),'xdata',xy(1,:),'ydata',xy(2,:));
            end
        
            for idx = 1:length(globalTraj)
                if idx > length(globalTraj)
                    break;
                end
                if ~isValid(idx)
                   set(obj.trajp11(idx), 'xdata',globalTraj(idx).Trajectory(:,1), 'ydata',globalTraj(idx).Trajectory(:,2));
                else
                set(obj.trajp(idx), 'xdata',globalTraj(idx).Trajectory(:,1), 'ydata',globalTraj(idx).Trajectory(:,2));
                end
            end
            set(obj.trajopt, 'xdata',optimalTrajectory(:,1), 'ydata',optimalTrajectory(:,2));
        end

        
        function [curState, futureTrajectory] = getActorInfo(obj)% 获取actors的信息
            curState = obj.curState;
            futureTrajectory = obj.futureTrajectory;
        end

        function update(obj, futureTrajectory) %actors位置和未来轨迹更新
            % 获取非ego的位姿和未来轨迹
            numActor=obj.numActor;
            curState = zeros(numActor,6);
            minUpdateSteps = (1/obj.replanRate)/obj.SampleTime;
            maxNumStates = obj.maxHorizon/obj.SampleTime;
            statesNeeded = max(maxNumStates-size(futureTrajectory(1).Trajectory,1),minUpdateSteps);

            for i = 1:statesNeeded
                poses = [];
                for k = 1:numActor
                    obj.actors{k}.s = obj.actors{k}.s + obj.SampleTime*obj.actors{k}.speed(1);
                    p1 = obj.actors{k}.refPath.interpolate(obj.actors{k}.s);
                    poses1.Position = [p1(1:2),0];
                    poses1.Velocity = obj.actors{k}.speed(1)*[cos(p1(3)),sin(p1(3)),0];
                    poses1.Yaw = p1(3)*180/pi;
                    poses1.AngularVelocity = [0,0,p1(4)*obj.actors{k}.speed(1)*180/pi];
                    poses = [poses,poses1];
                end

                for j = 1:numActor
                    actIdx = j;
                    xy = poses(actIdx).Position(1:2);
                    v  = norm(poses(actIdx).Velocity,2);
                    th = atan2(poses(actIdx).Velocity(2),poses(actIdx).Velocity(1));
                    k = poses(actIdx).AngularVelocity(3)/v/180*pi;
                    futureTrajectory(j).Trajectory(i,:) = [xy th k v 0];
                end
            end

            % Reorder the states
            for i = 1:numActor
                futureTrajectory(i).Trajectory = circshift(futureTrajectory(i).Trajectory,-statesNeeded,1);
                curState(i,:) = futureTrajectory(i).Trajectory(1,:);
            end

            obj.futureTrajectory = futureTrajectory;
            obj.curState = curState;
        end


        function show(obj) % 绘图
            % 绘制场景
            hold on;
            ss = 0:1:obj.refPath.Length;
            len = length(ss);
            pp = obj.refPath.interpolate(ss);
            plot(pp(:,1),pp(:,2), '--');

            for i = 3:5
                frestate = [0, 0, 0, -obj.laneWidth/2+obj.laneWidth*(i-3), 0, 0];
                frestates = repmat(frestate, len, 1);
                frestates(:,1) = ss';
                pp1 = obj.refPath.frenet2global(frestates);
                plot(pp1(:,1),pp1(:,2), 'k','LineWidth',1);
            end
        axis equal; 
       % axis([110 180,40,60]);
       % set(gca, 'DataAspectRatio', [1 1 1]);
       % set(gcf, 'unit', 'centimeters', 'position', [5 5 20 10]);


            obj.fig = gcf;
            obj.ax = gca;
            obj.egop = patch(nan,nan,'r','FaceColor','none' ,'EdgeColor','red','LineWidth',2);
            obj.carp = zeros(5,1);

            for i = 1:5
                obj.carp(i) = patch(nan,nan,'r','FaceColor','none' ,'EdgeColor','blue','LineWidth',2);
                obj.cartraj(i)=plot(nan,nan,'--b','linewidth',1.0);
            end


            obj.trajp = zeros(20,1);
            for i = 1:100
                obj.trajp(i) = plot(nan,nan,'--g','linewidth',1.5);
            end
            obj.trajp11 = zeros(20,1);
            for i = 1:100
                obj.trajp11(i) = plot(nan,nan,'--r','linewidth',1.5);
            end

           obj.trajopt = plot(nan,nan, 'marker','.', 'color','r','linewidth',1.5);
            title('行驶轨迹','FontSize',15);

            xlabel('横向位置[m]','Interpreter','none','FontSize',15);
            ylabel('纵向位置[m]','Interpreter','none','FontSize',15);
            ylim([40 60])
            xlim([100 200])
         
        end


        function obj = ScenarioEnv()% 构造函数
            waypoints = [0 50; 150 50; 300 50; 450 50; 1000 50]; % in meters
            
            obj.laneWidth   = 3.6;
            obj.refPath = FrenetReferencePath(waypoints);
            obj.numActor=2;

            % Add actors
            car1.Position = [50 50 0];
            car1.waypoints = [50  50 0; 500 50 0;];
            car1.speed = 10*ones(10,1);
            
            car2.Position = [80 53.6 0];
            car2.waypoints = [80 53.6 0;  1000 53.6 0;];
            car2.speed = 36*ones(10,1);

            car3.Position =  [100 53.6 0];
            car3.waypoints = [100 53.6 0; 500 53.6 0];
            car3.speed =12*ones(10,1);

           
            car5.Position =[180 50.0 0];
            car5.waypoints = [180 50.0 0; 1000 50.0 0;];    
            car5.speed = 18*ones(10,1);
% 
             car1.refPath = FrenetReferencePath(car1.waypoints(:,1:2));
             car2.refPath = FrenetReferencePath(car2.waypoints(:,1:2));
             car3.refPath = FrenetReferencePath(car3.waypoints(:,1:2));
             car5.refPath = FrenetReferencePath(car5.waypoints(:,1:2));

             car1.s = 0;
             car2.s = 0;
             car3.s = 0;
             car5.s = 0;
             obj.actors = {car2,car5};

        end
    end

end