function results = runArtiSynthParallel2()
    % Define the objective function for Bayesian optimization
    function objective = objectiveFunction(params)
        % Set up the ArtiSynth environment on the worker
        setupArtiSynthWorker();
        
        % Extract parameters (example: x and y)
        x = params.x;
        y = params.y;
        
        % Initialize default objective as Inf (penalty for errors)
        objective = Inf;
        
        % Run the ArtiSynth model without GUI on this worker
        try
            % Add necessary Java paths for ArtiSynth
            javaaddpath(fullfile(getenv('ARTISYNTH_HOME'), 'lib', 'jython-2.7.3.jar'));
            
            % Initialize ArtiSynth model and set parameters
            ah1 = artisynth('-noGui', '-model', 'artisynth.demos.tutorial.FourBarLinkage');

            % Placeholder for actual parameter setting and simulation
            % Example: Set parameters and run simulation
            % ah1.setSomeParameter(x, y); 
            % Example: Extract the objective value
            % Replace the following line with actual method to get output metric
            objective = simulateArtiSynthAndGetObjective(ah1, x, y);

            % Exit ArtiSynth cleanly
            ah1.quit();
        catch ME
            disp(['Error during execution: ', ME.message]);
        end
    end

    % Define the optimization variables and ranges
    vars = [optimizableVariable('x', [0, 1]), optimizableVariable('y', [0, 1])];

    % Perform Bayesian optimization
    results = bayesopt(@objectiveFunction, vars, ...
                       'IsObjectiveDeterministic', false, ...
                       'UseParallel', true, ...
                       'MaxObjectiveEvaluations', 30);
end

% Setup function to be called on each worker
function setupArtiSynthWorker()
    % Add necessary paths
    addpath(fullfile(getenv('ARTISYNTH_HOME'), 'matlab'));
    % Set the ArtiSynth classpath
    setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
end

% Placeholder for simulation execution and objective extraction
function objective = simulateArtiSynthAndGetObjective(~, x, y)
    % Placeholder code: Replace with actual simulation setup and execution
    % Example: Set parameters on the model, run the simulation, and extract output metric
    % Simulated dummy objective for illustration
    objective = x + y; % Replace with actual computation
end
