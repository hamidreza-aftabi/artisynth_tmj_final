% Define the objective function
function loss = runArtisynthSim(params)
    zOffset = double(params.zOffset);
    
    % Run the Artisynth simulation with the specified zOffset
    ah = artisynth('-model', 'artisynth.istar.reconstruction.MandibleRecon');
    root = ah.root();
    root.createFibulaOptimization(zOffset);

    % Perform the simulation steps
    for i = 1:100
        ah.step();
    end

    % Pause to allow processes to finish
    pause(5);

    % Create screws and export files
    root.getPlatePanel.createScrews();
    root.exportFiles();
    root.exportFemPlate();

    % Close Artisynth
    ah.quit();

    % Return a fixed loss value
    loss = 1;
end