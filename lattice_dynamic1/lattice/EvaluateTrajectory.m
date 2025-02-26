function isValid = exampleHelperEvaluateTrajectory(globalTrajectory,tra_err)
%exampleHelperEvaluateTrajectory Evaluate trajectory constraints.
%
%   This function is for internal use only. It may be removed in the future

%   ISVALID = exampleHelperEvaluateTrajectory(GLOBALTRAJECTORY, MAXACCELERATION, MAXCURVATURE, MINVELOCITY)
%   Takes in a N-element struct array of trajectories, GLOBALTRAJECTORY, and
%   checks whether each trajectory violates the MAXACCELERATION, MAXCURVATURE,
%   or MINVELOCITY constraints.
%
%   Each element of GLOBALTRAJECTORY contains the fields Trajectory, an 
%   N-by-[x y theta kappa v a] matrix, and Times, an N-by-1 vector of timesteps.
%
%   Returns an N-by-1 vector of logicals, ISVALID, where an entry of false 
%   means that the corresponding trajectory violated one or more constraints,
%   and true means that all constraints were met.
%
% Copyright 2020 The MathWorks, Inc.

global planningConstraints
    isValid = true(numel(globalTrajectory),1);
    for i = 1:numel(globalTrajectory)
        % Acceleration constraint
        accViolated  = any(abs(globalTrajectory(i).Trajectory(:,6)) > abs(planningConstraints.maxAcceleration));

        % Curvature constraint
        curvViolated = any(abs(globalTrajectory(i).Trajectory(:,4)) > abs(planningConstraints.maxCurvature));

        % Velocity constraint
 

        velViolated =  any(globalTrajectory(i).Trajectory(:,5) < planningConstraints.minVelocity);

        isValid(i) = ~velViolated;
    end
end