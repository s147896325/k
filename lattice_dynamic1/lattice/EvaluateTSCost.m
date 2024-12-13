function [costs] = EvaluateTSCost(egoFrenetState,terminalStates, times,globalTraj, frenetTraj,actorPoses)
%exampleHelperEvaluateTSCost Evaluate trajectory cost.
%
%   This function is for internal use only. It may be removed in the future

%   COSTS = exampleHelperEvaluateTSCost(TERMINALSTATES, times, laneWidth, speedLimit, speedWeight, latWeight, timeWeight)
%   Evaluates the cost of an N-by-6 matrix of Frenet states, TERMINALSTATES,
%   and corresponding N-by-1 vector of timespans, TIMES.
%
%   Cost is based on lateral deviation from a lane center, calculated using
%   LANEWIDTH, deviation from the desired velocity, SPEEDLIMIT, and the span
%   of time required by the trajectory, TIMES.
%
%   Each of these metrics is scaled by the accompanying weights,
%   LATWEIGHT, SPEEDWEIGHT, and TIMEWEIGHT, respectively.
%
% Copyright 2020 The MathWorks, Inc.
global  laneWidth  latDevWeight  numActors Driving_riskObj  speedLimit  safetyTH

% Determine current and future lanes

%futureLane = PredictLane(egoFrenetState, laneWidth, 0);
%lateralOffsets = (2-futureLane+.5)*laneWidth;
%latDeviation = abs(lateralOffsets-terminalStates(:,4));

%latCost = latDeviation*latDevWeight/3.6;

% Calculate trajectory time cost
%timeCost = times*timeWeight;

% Calculate terminal speed vs desired speed cost
%speedCost = abs(terminalStates(:,2)-speedLimit)*speedWeight;

%% risk metric
J_obst=zeros(numel(times),1);

curvature=zeros(numel(times),2);

gamma=1.0;

J_obst_tra=repmat(struct('Trajectory',[]), 40, 1);

current_numActors=[];

for actidx=1:numActors
     if(actorPoses(actidx).States(1,1)<=egoFrenetState(1,1)+egoFrenetState(1,2)*safetyTH&&actorPoses(actidx).States(1,1)>=egoFrenetState(1,1)-50)
        current_numActors=[current_numActors;actidx];
     end
end

for i=1:numel(times)
    J_obst_point=[];
    if(isempty(current_numActors))
        ;
    else
        for j=1:(numel(globalTraj(i).Trajectory(:,1))-1)

            Driving_riskObj.Hv_states=[globalTraj(i).Trajectory(j+1,1),globalTraj(i).Trajectory(j+1,2),globalTraj(i).Trajectory(j+1,5)];

            for actidx =1:length(current_numActors)
                Driving_riskObj.Obs_states{actidx}=[actorPoses(current_numActors(actidx)).States(j,1),actorPoses(current_numActors(actidx)).States(j,2),actorPoses(current_numActors(actidx)).States(j,4)];
            end

            point_trfficrisk=gamma^j*Driving_riskObj.Total_drf();
            J_obst(i,1)=J_obst(i,1)+point_trfficrisk;
        end
    end

end


speedCost = abs(terminalStates(:,2)-speedLimit);
minspeedCost = min(speedCost);
maxspeedCost = max(speedCost);
normalizedspeedCost = (speedCost - minspeedCost) / (maxspeedCost - minspeedCost);
if all(J_obst == 0)
    normalizedrisk=zeros(numel(times),1);
else
  
    minrisk = min(J_obst);
    maxrisk = max(J_obst);
    normalizedrisk = (J_obst - minrisk) / (maxrisk - minrisk);
end


    curLane = PredictLane(egoFrenetState, laneWidth, 0);
    lateralOffsets2 = (2-curLane)*laneWidth;
    latDeviation2 = abs(lateralOffsets2-terminalStates(:,4));
    latCost2 = latDeviation2/laneWidth

%Return cumulative cost
costs = normalizedrisk*8+normalizedspeedCost*2+latCost2


end