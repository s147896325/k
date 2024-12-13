function [terminalStates, times] = SamplingBasicLaneChange(refPath, laneWidth, egoState, dt)   
    

if egoState(5) == 0
        terminalStates = [];
        times = [];
else
        % Convert ego state to Frenet coordinates
        frenetState = global2frenet(refPath, egoState);

        % Get current lane
        curLane = PredictLane(frenetState, laneWidth, 0);

        % Determine if future lanes are available
 adjacentLanes = curLane+[2:-1:-2];
        validLanes = adjacentLanes >=2 & adjacentLanes <=3;

        % Calculate lateral deviation for adjacent lanes
        lateralOffset = (2-adjacentLanes(validLanes)+1)*laneWidth;
        numLane = nnz(validLanes);

        % Calculate terminal states
        terminalStates = zeros(numLane*numel(dt),6);
        terminalStates(:,1) = nan;
        terminalStates(:,2) = egoState(5)-1;
        terminalStates(:,4) = repelem(lateralOffset(:),numel(dt),1);
        times = repmat(dt(:),numLane,1);
    end
end