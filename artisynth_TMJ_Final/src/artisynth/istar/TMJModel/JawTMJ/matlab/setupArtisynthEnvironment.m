% Function to set up environment on workers
function setupArtisynthEnvironment()
    fprintf('Setting up ArtiSynth environment\n');
    artHome = getenv('ARTISYNTH_HOME');
    fprintf('ARTISYNTH_HOME: %s\n', artHome);

    if isempty(artHome) || ~exist(artHome, 'dir')
        error('ARTISYNTH_HOME is not correctly set or does not exist');
    end

    addpath(fullfile(artHome, 'matlab'));
    fprintf('Added ArtiSynth core to MATLAB path.\n');
    maspackDir = fullfile(artHome, 'classes', 'maspack');
    javaaddpath(maspackDir);
    fprintf('Added maspack classes to Java classpath: %s\n', maspackDir);

    fprintf('Current MATLAB path:\n');
    disp(path);

    setArtisynthClasspath(artHome);
    fprintf('Set ArtiSynth classpath.\n');

    % Additional logging for Java classpath and other diagnostics
    jcp = javaclasspath('-dynamic');
    fprintf('Current dynamic Java classpath:\n');
    disp(jcp);
end

% Example objective function for Bayesian optimization
function objective = Paralell_Test_Function(params)
    try
        zOffset = params.zOffset;
        leftRoll = params.leftRoll;
        leftPitch = params.leftPitch;
        rightRoll = params.rightRoll;
        rightPitch = params.rightPitch;

        fprintf('Evaluating with parameters: zOffset = %.2f, leftRoll = %.2f, leftPitch = %.2f, rightRoll = %.2f, rightPitch = %.2f\n', ...
                zOffset, leftRoll, leftPitch, rightRoll, rightPitch);
        
        % Initialize ArtiSynth model
        fprintf('Starting ArtiSynth model initialization\n');
        ah1 = artisynth('-model', 'artisynth.demos.tutorial.FourBarLinkage');

        % Check if ah1 is properly initialized
        if isempty(ah1)
            error('Failed to initialize ArtiSynth model');
        end

        fprintf('ArtiSynth model initialized, calculating objective\n');
        objective = calculateObjective(ah1);
        fprintf('Objective calculated: %.2f\n', objective);

        % Clean up
        fprintf('Closing ArtiSynth model\n');
        close(ah1);
    catch ME
        fprintf('Error occurred: %s\n', ME.message);
        rethrow(ME);
    end
end

% Placeholder function to calculate the objective value
function objValue = calculateObjective(model)
    objValue = rand(); % Placeholder
end
