clc
clear all
close all

%import maspack.matrix.*

addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
setArtisynthClasspath (getenv ('ARTISYNTH_HOME'));

% Check for the Parallel Computing Toolbox    
%if isempty(ver('parallel'))
%        error('Parallel Computing Toolbox is not installed.');
%end

% Open a parallel pool
%pool = parpool;

    % Set up Java classpath for parallel workers
 %spmd
 %   addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
 %   setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
 % end

% Define the optimizable variable
zOffsetVar = optimizableVariable('zOffset', [0.5, 2.0]);

% Run Bayesian optimization
results = bayesopt(@runArtisynthSim, zOffsetVar, ...
    'Verbose', 1, ...
    'AcquisitionFunctionName', 'expected-improvement-plus', ...
    'MaxObjectiveEvaluations', 5, ... % Reduced for testing purposes
    'UseParallel', false); % Enable parallel evaluations if possible

% Get the best parameters
bestParams = bestPoint(results);

% Display the best parameters
disp('Best Parameters:');
disp(['zOffset: ', num2str(bestParams.zOffset)]);

% Clean up parallel pool after optimization
%delete(gcp('nocreate'));
