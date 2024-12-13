function [allTS, allDT, numTS] = SamplingEndcontions(refPath, laneWidth, egoState )
    global  speedLimit timeHorizons
  % Generate cruise control states.
    
    %[termStatesCC,timesCC] = SamplingBasicCruiseControl(refPath,laneWidth,egoState,speedLimit,timeHorizons);
 

    % Generate lane change states.
    [termStatesLC6,timesLC6] = SamplingBasicLaneChange6(refPath,laneWidth,egoState,timeHorizons);

    % Generate lane change states.
    [termStatesLC1,timesLC1] = SamplingBasicLaneChange1(refPath,laneWidth,egoState,timeHorizons);
    [termStatesLC2,timesLC2] = SamplingBasicLaneChange2(refPath,laneWidth,egoState,timeHorizons);

    [termStatesLC,  timesLC] = SamplingBasicLaneChange(refPath,laneWidth,egoState,timeHorizons);

    [termStatesLC3,timesLC3] = SamplingBasicLaneChange3(refPath,laneWidth,egoState,timeHorizons);

    [termStatesLC4,timesLC4] = SamplingBasicLaneChange4(refPath,laneWidth,egoState,timeHorizons);
    [termStatesLC5,timesLC5] = SamplingBasicLaneChange5(refPath,laneWidth,egoState,timeHorizons);

    
    % Generate vehicle following states.
    %[termStatesF,timesF] = SamplingBasicLeadVehicleFollow(refPath,laneWidth,safetyGap,egoState,curActorState,timeHorizons);
    
    % Combine the terminal states and times.
    allTS = [ termStatesLC6;termStatesLC5;termStatesLC4;termStatesLC3;termStatesLC;  termStatesLC1;     termStatesLC2;     ];
    allDT = [  timesLC2;          timesLC1;       timesLC;        timesLC3;        timesLC4;        timesLC5; timesLC6;];
    numTS = [ numel(timesLC2);numel(timesLC1);numel(timesLC); numel(timesLC3); numel(timesLC4); numel(timesLC5); numel(timesLC6);];


end