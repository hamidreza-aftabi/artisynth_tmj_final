function runArtiSynthParallel()
    % Set up the MATLAB environment for ArtiSynth on all workers
    % and run the ArtiSynth model in parallel

    % Setup function to be called on each worker
    function setupArtiSynthWorker()
        % Add necessary paths
        addpath(fullfile('..', '..', '..', '..', '..', '..', '..', '..', 'artisynth_core', 'matlab'));
        % Set the ArtiSynth classpath
        setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
    end

    % Start the parallel pool with a specified number of workers
    pool = parpool('local', 4); % Adjust the number of workers as needed

    % Broadcast the setup function to all workers
    f = parfevalOnAll(@setupArtiSynthWorker, 0); % 0 outputs
    wait(f); % Wait for setup to complete on all workers

    % Run the ArtiSynth model in parallel without GUI
    try
        parfor i = 1:4
            % Initialize ArtiSynth model on each worker
            try
                %javaaddpath(fullfile(getenv('ARTISYNTH_HOME'), 'lib', 'jython-2.7.3.jar'));
                artisynth('-noGui', '-model', 'artisynth.demos.tutorial.FourBarLinkage');
            catch ME
                disp(['Error on worker ', num2str(i), ': ', ME.message]);
            end
        end
    catch ME
        disp(['Error during parallel execution: ', ME.message]);
    end

    % Shutdown the parallel pool
    delete(pool);
end
