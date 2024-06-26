function loss = runArtisynthSimTwo(params)
    
    zOffset = double(params.zOffset);
    leftRoll = double(params.leftRoll);
    leftPitch = double(params.leftPitch);
    rightRoll = double(params.rightRoll);
    rightPitch = double(params.rightPitch);

    % Debugging information
    fprintf('Running simulation with zOffset = %.2f, leftRoll = %.2f, leftPitch = %.2f, rightRoll = %.2f, rightPitch = %.2f\n', ...
        zOffset, leftRoll, leftPitch, rightRoll, rightPitch);
    fprintf('ARTISYNTH_HOME = %s\n', getenv('ARTISYNTH_HOME'));

    % Calculate the new resection planes
    % Left Plane
    init_axis_l = [-0.89201, -0.45202, 0];
    init_angle_l =  84.33;
    [new_axis_l, new_angle_l] = rotate_axis_angle_around_local_x(init_axis_l, init_angle_l, leftRoll);
    [newer_axis_l, newer_angle_l] = rotate_axis_angle_around_local_y(new_axis_l, new_angle_l, leftPitch);

    plane_normal_l = find_plane_normal_from_axis_angle(newer_axis_l, newer_angle_l);

    % Right Plane
    init_axis_r = [0.8866, 0.46253, 0];
    init_angle_r = 100.54;
    [new_axis_r, new_angle_r] = rotate_axis_angle_around_local_x(init_axis_r, init_angle_r, rightRoll);
    [newer_axis_r, newer_angle_r] = rotate_axis_angle_around_local_y(new_axis_r, new_angle_r, rightPitch);

    plane_normal_r = find_plane_normal_from_axis_angle(newer_axis_r, newer_angle_r);

    % Change resection file
    input_filename = 'resection_plane_initial.txt';
    output_filename = 'resection_plane_optimize.txt';

    modify_normals_in_file_two(input_filename, output_filename, plane_normal_l, plane_normal_r);
   

    % Set up Artisynth environment and run simulation
    try
        ah = artisynth('-model', 'artisynth.istar.reconstruction.MandibleRecon');
        if isempty(ah)
            error('Failed to initialize Artisynth.');
        end
    catch ME
        disp('Error during ArtiSynth initialization:');
        disp(ME.message);
        rethrow(ME);
    end
    
    root = ah.root();
    root.createFibulaOptimization(zOffset);

    % Perform simulation steps
    for i = 1:100
        ah.step();
    end

    % Pause to allow processes to finish
    pause(6);

    % Create screws and export files
    root.getPlatePanel.createScrews();
    root.exportFiles();
    root.exportFemPlate();

    sourceDir = 'C:\Users\Hamidreza\git\artisynth_istar\src\artisynth\istar\reconstruction\optimizationResultTwo';
    destinationDir = 'C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry';

    fileList = {'donor_opt.obj', 'plate_opt.art', 'plate_opt.obj', 'resected_mandible_l_opt.obj', 'resected_mandible_r_opt.obj', 'screw_opt.obj'};

    for i = 1:length(fileList)
        sourceFile = fullfile(sourceDir, fileList{i});
        copyfile(sourceFile, destinationDir);
    end

    % Close the first Artisynth instance
    ah.quit();
    ah = [];
    java.lang.System.gc();

    % Run PyMeshLab remeshing script
    pyrunfile('PymeshlabRemesh.py');
    
    pause(6);

    % Run the second Artisynth model
    try
        ah1 = artisynth('-model', 'artisynth.istar.TMJModel.JawTMJ.JawFemDemoOptimize');
        if isempty(ah1)
            error('Failed to initialize the second Artisynth instance.');
        end
    catch ME
        disp('Error during second ArtiSynth initialization:');
        disp(ME.message);
        rethrow(ME);
    end
    
    for i = 1:1400
        ah1.step();
    end

    left_percent = ah1.getOprobeData('6');
    right_percent = ah1.getOprobeData('7');

    % Calculate the loss
    loss = - (mean(left_percent(:,2)) + mean(right_percent(:,2))) / ...
            (mean(left_percent(:,2)) - mean(right_percent(:,2)) + 1e-7);

    % Close the second Arisynth instance
    pause(3);
    ah1.quit();
    ah1 = [];
    java.lang.System.gc();
end
