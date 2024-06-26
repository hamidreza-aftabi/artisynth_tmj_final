% Main script: mainScript.m

clc
clear all

addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
setArtisynthClasspath (getenv ('ARTISYNTH_HOME'));


% Path to save the optimization results
resultsFile = 'bayesoptResults.mat';

% Number of total iterations
totalIterations = 25;

% Load previous results if they exist
if isfile(resultsFile)
    load(resultsFile, 'results', 'currentIteration');
else
    results = [];
    currentIteration = 1;
end

% Perform optimization in iterations
while currentIteration <= totalIterations
    disp(['Starting iteration: ' num2str(currentIteration)]);
    
    % Run the helper function
    results = helperFunction(results, currentIteration);
    
    % Save results and current iteration
    save(resultsFile, 'results', 'currentIteration');
    
    % Increment the iteration count
    currentIteration = currentIteration + 1;
    
    % Check if we need to continue and restart MATLAB
    if currentIteration <= totalIterations
        disp('Restarting MATLAB to manage memory...');
        % Save workspace and restart MATLAB
        save(resultsFile, 'results', 'currentIteration', 'totalIterations');
        % Use system command to restart MATLAB
        matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
        matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, mfilename);
        system(matlabCommand);
        exit; % Close the current MATLAB session
    end
end

disp('Optimization process finished.');
