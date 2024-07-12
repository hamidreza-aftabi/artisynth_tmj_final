clc;
clear all;

resultsFile = 'bayesoptResults_25_TMJ_FIXED_Trial7_Costhalf.mat';


%addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
addpath(fullfile('..','..', '..', '..', '..', '..', '..', '..', 'artisynth_core', 'matlab'));

setArtisynthClasspath(getenv('ARTISYNTH_HOME'));

% Number of total iterations
totalIterations = 50;

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
  
    try
        % Run the helper function
        results = helperFunctionTwo(results, currentIteration);
        
        % Save results and current iteration
        currentIteration = currentIteration + 1;
        save(resultsFile, 'results', 'currentIteration');
        
        % Check if we need to continue and restart MATLAB
        if currentIteration <= totalIterations
            disp('Restarting MATLAB to manage memory...');
            % Save workspace and restart MATLAB
            save(resultsFile, 'results', 'currentIteration', 'totalIterations');
            % Use system command to restart MATLAB
            matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
            matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, mfilename('fullpath'));
            system(matlabCommand);
            exit; % Close the current MATLAB session
        end
    catch ME
        % Handle specific Java exception
        if strcmp(ME.identifier, 'MATLAB:Java:GenericException')
            javaEx = ME.ExceptionObject;
            if isa(javaEx, 'maspack.matrix.NumericalException') && contains(javaEx.message, 'Inverted elements')
                disp('NumericalException: Inverted elements error occurred:');
                disp(getReport(javaEx));
            else
                disp(['Java exception occurred: ' ME.message]);
            end
        else
            disp(['Error occurred: ' ME.message]);
            disp('MATLAB Error...');
            % Save workspace and restart MATLAB
            save(resultsFile, 'results', 'currentIteration', 'totalIterations');
            % Use system command to restart MATLAB
            matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
            matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, mfilename('fullpath'));
            system(matlabCommand);
            exit; % Close the current MATLAB session
        end
        
    end
end
