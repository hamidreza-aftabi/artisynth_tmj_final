clc;
clear all;

% Add the required paths and set class paths for ArtiSynth
addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
setArtisynthClasspath(getenv('ARTISYNTH_HOME'));

% Load or initialize the simulation parameters and state
resultsFile = 'five_sensitivity_offset.mat';
if exist(resultsFile, 'file')
    load params_five_offset.mat;
    load(resultsFile, 'sensitivity_result', 'currentIteration', 'totalIterations');
else
    % Load initial parameters from the .mat file if starting fresh
    load params_five_offset.mat;
    sensitivity_result = struct('losses', [], 'left_percents', [], 'right_percents', []);
    currentIteration = 0;
    totalIterations = 5;  % Define the total number of iterations here
end

% Check if all iterations are complete
if currentIteration < totalIterations
    currentIteration = currentIteration + 1;
    
    % Run the simulation
    [loss, left_percent, right_percent] = runArtisynthSimManual(params);
    sensitivity_result.losses = [sensitivity_result.losses, loss];
    sensitivity_result.left_percents{end+1} = left_percent;
    sensitivity_result.right_percents{end+1} = right_percent;
    
    % Save all results and the current state
    save(resultsFile, 'sensitivity_result', 'currentIteration', 'totalIterations');
    
    % Restart MATLAB to manage memory
    disp('Restarting MATLAB to manage memory...');
    matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
    matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, mfilename);
    system(matlabCommand);
    exit;  % Close the current MATLAB session
else
    disp('All iterations complete. Processing results...');
    % Insert any post-processing code here if necessary
end
