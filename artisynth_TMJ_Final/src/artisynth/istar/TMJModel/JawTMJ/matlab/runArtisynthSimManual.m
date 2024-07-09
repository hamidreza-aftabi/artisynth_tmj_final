    
    addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
    setArtisynthClasspath(getenv('ARTISYNTH_HOME'));

    sourceDir = 'C:\Users\Hamidreza\git\artisynth_istar\src\artisynth\istar\reconstruction\optimizationResult';
    destinationDir = 'C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry';

    bodyList ="C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry\bodyList.txt";
    toggleComment(bodyList, 'screw1', 'add');

    toggleComment('bodyList.txt', 'screw1', 'add');

    num_screws = 1;
    num_segment = 1;

    zOffset = double(params.zOffset);
    leftRoll = double(params.leftRoll);
    leftPitch = double(params.leftPitch);
    rightRoll = double(params.rightRoll);
    rightPitch = double(params.rightPitch);

    % Debugging information
    fprintf('Running simulation with zOffset = %.2f, leftRoll = %.2f, leftPitch = %.2f, rightRoll = %.2f, rightPitch = %.2f\n', ...
        zOffset, leftRoll, leftPitch, rightRoll, rightPitch);
    fprintf('ARTISYNTH_HOME = %s\n', getenv('ARTISYNTH_HOME'));

    % Calculate the new resection plane

    % Left Plane
    init_axis_l = [-0.37445 -0.82382 -0.42556];
    init_angle_l = 100.22;
   

    %plane_normal_l = find_plane_normal_from_axis_angle(newer_axis_l, newer_angle_l);

    % Right Plane
    init_axis_r = [0.67407 -0.37913 0.63395];
    init_angle_r = 144.41;

    %plane_normal_r = find_plane_normal_from_axis_angle(newer_axis_r, newer_angle_r);

    % Change resection file
    %input_filename = 'resection_plane_initial.txt';
    %output_filename = 'resection_plane_optimize.txt';

    %modify_normals_in_file(input_filename, output_filename, plane_normal_l, plane_normal_r);
   

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
   
    root.getPlateBuilder().setNumScrews (num_screws);
    root.getSegmentGenerator.setMaxSegments(num_segment);
    root.getSegmentGenerator.setNumSegments (num_segment);

    root.importFibulaOptimization();
    
    import maspack.matrix.AxisAngle ;

    planeL = ah.find('models/Reconstruction/resectionPlanes/planeL');
    planeL.setOrientation(AxisAngle ([init_axis_l, deg2rad(init_angle_l)]));

    planeR = ah.find('models/Reconstruction/resectionPlanes/planeR');
    planeR.setOrientation(AxisAngle ([init_axis_r, deg2rad(init_angle_r)]));

    ah.step();
    
    pause(1);

    [new_axis_l, new_angle_l] = rotate_axis_angle_around_local_x(init_axis_l, init_angle_l, leftRoll);
    [newer_axis_l, newer_angle_l] = rotate_axis_angle_around_local_y(new_axis_l, new_angle_l, leftPitch);
    planeL.setOrientation(AxisAngle ([newer_axis_l, deg2rad(newer_angle_l)]));

    [new_axis_r, new_angle_r] = rotate_axis_angle_around_local_x(init_axis_r, init_angle_r, rightRoll);
    [newer_axis_r, newer_angle_r] = rotate_axis_angle_around_local_y(new_axis_r, new_angle_r, rightPitch);
    planeR.setOrientation(AxisAngle ([newer_axis_r, deg2rad(newer_angle_r)]));

    ah.step();

    pause(1);

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

    
    fileList = {'donor_opt0.obj', 'plate_opt.art', 'resected_mandible_l_opt.obj', 'resected_mandible_r_opt.obj', 'screw_opt0.obj'};

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

    left_percent = ah1.getOprobeData('5');
    right_percent = ah1.getOprobeData('6');

    % Calculate the loss
    %loss = - (mean(left_percent(:,2)) + mean(right_percent(:,2))) / ...
    %        abs(mean(left_percent(:,2)) - mean(right_percent(:,2)) + 1e-7);

    loss = - (0.5*(mean(left_percent(:,2)) + mean(right_percent(:,2))) - 0.5 *abs(mean(left_percent(:,2)) - mean(right_percent(:,2)))) ;

    % Close the second Arisynth instance
    pause(3);
    ah1.quit();
    ah1 = [];
    java.lang.System.gc();
    
