clc;
clear all;
close all;

% Define the path to ArtiSynth
artisynthHome = 'C:\Users\Hamidreza\git\artisynth_core';

% Set ARTISYNTH_HOME environment variable
setenv('ARTISYNTH_HOME', artisynthHome);

% Check if ARTISYNTH_HOME is set correctly
if isempty(getenv('ARTISYNTH_HOME'))
    error('ARTISYNTH_HOME environment variable is not set.');
else
    disp(['ARTISYNTH_HOME is set to: ', getenv('ARTISYNTH_HOME')]);
end

% Add ArtiSynth MATLAB path
addpath(fullfile(getenv('ARTISYNTH_HOME'), 'matlab'));

% Verify that the path exists
if ~isfolder(fullfile(getenv('ARTISYNTH_HOME'), 'matlab'))
    error('The specified ARTISYNTH_HOME/matlab directory does not exist.');
end

% Set up ArtiSynth classpath
try
    setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
    disp('ArtiSynth classpath set successfully.');
catch ME
    disp('Error setting ArtiSynth classpath:');
    disp(ME.message);
    rethrow(ME);
end

% Ensure native libraries are on the library path
setenv('PATH', [getenv('PATH') ';' fullfile(getenv('ARTISYNTH_HOME'), 'lib', 'Windows64')]);

% Verify that the library path exists
if ~isfolder(fullfile(getenv('ARTISYNTH_HOME'), 'lib', 'Windows64'))
    error('The specified ARTISYNTH_HOME/lib/Windows64 directory does not exist.');
end

% Check for the Parallel Computing Toolbox
if isempty(ver('parallel'))
    error('Parallel Computing Toolbox is not installed.');
else
    disp('Parallel Computing Toolbox is installed.');
end

% Open a parallel pool
pool = parpool;

% Set up Java classpath for parallel workers
spmd
    addpath(fullfile(getenv('ARTISYNTH_HOME'), 'matlab'));
    setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
end

% Define the optimizable variable
zOffsetVar = optimizableVariable('zOffset', [-4, 3]);


% Load the previous optimization results
load('bayesoptResults1.mat', 'results');

% Extract the best points from previous results
initialX = results.XTrace;
initialObjective = results.ObjectiveTrace;

% Define initial points structure
initialPoints = table(initialX.zOffset, 'VariableNames', {'zOffset'});

% Resume Bayesian optimization with initial points
results = bayesopt(@runArtisynthSim, zOffsetVar, ...
    'Verbose', 1, ...
    'AcquisitionFunctionName', 'expected-improvement', ...
    'MaxObjectiveEvaluations', 2, ... % Additional evaluations
    'GPActiveSetSize', 300, ... % Use active set to speed up Gaussian Process calculations
    'IsObjectiveDeterministic', true, ... % Set if the objective function is deterministic
    'UseParallel', false, ... % Enable parallel evaluations
    'InitialX', initialPoints, ...
    'InitialObjective', initialObjective);

% Get the best parameters
bestParams = bestPoint(results);

% Display the best parameters
disp('Best Parameters:');
disp(['zOffset: ', num2str(bestParams.zOffset)]);

save('bayesoptResults2.mat', 'results');

% Clean up parallel pool after optimization
delete(gcp('nocreate'));

