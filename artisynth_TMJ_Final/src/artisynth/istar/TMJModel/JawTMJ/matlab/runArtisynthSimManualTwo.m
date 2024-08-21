    
    
    defectType = "RB"; 
    

    %addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
    %setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
    addpath(fullfile('..','..', '..', '..', '..', '..', '..', '..', 'artisynth_core', 'matlab'));
    setArtisynthClasspath(getenv('ARTISYNTH_HOME'));    

    %sourceDir = 'C:\Users\Hamidreza\git\artisynth_istar\src\artisynth\istar\reconstruction\optimizationResultTwo';
    %destinationDir = 'C:\\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry';
    sourceDir = fullfile('..','..','..', '..', '..', '..', '..', '..', 'artisynth_istar', 'src', 'artisynth', 'istar', 'reconstruction', 'optimizationResultTwo');
    destinationDir = fullfile('..', 'geometry');

    %bodyList ="C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry\bodyList.txt";
    bodyList = fullfile('..', 'geometry', 'bodyList.txt');

    resetMuscles();
    
    toggleComment(bodyList, 'screw1', 'remove');

    if defectType == "RB"
        
        removeRBMuscles();

    end


    num_screws = 2;
    num_segment = 2;

    %zOffset = double(params.zOffset);
    zOffset = 0
    %leftRoll = double(params.leftRoll);
    leftRoll= -0.301187399385210
    %leftPitch = double(params.leftPitch);
    leftPitch = 10.743228307430662
    %rightRoll = double(params.rightRoll);
    rightRoll = 11.793802384125314
    %rightPitch = double(params.rightPitch);
    rightPitch = -14.066058915026181 


    % Debugging information
    fprintf('Running simulation with zOffset = %.2f, leftRoll = %.2f, leftPitch = %.2f, rightRoll = %.2f, rightPitch = %.2f\n', ...
        zOffset, leftRoll, leftPitch, rightRoll, rightPitch);
    fprintf('ARTISYNTH_HOME = %s\n', getenv('ARTISYNTH_HOME'));

    % Calculate the new resection plane

    % Left Plane
    %init_axis_l = [-0.37445 -0.82382 -0.42556];
    init_axis_l = [-0.35978 -0.83742 -0.41145];

    %init_angle_l = 100.22;
    init_angle_l = 99.373;


    % Right Plane
    init_axis_r = [0.74092 -0.52003 0.42498];
    init_angle_r = 149.41;

    

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

    root.importFibulaOptimizationTwo();
    
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


    [new_axis_l, new_angle_l] = rotate_axis_angle_around_local_x(init_axis_l, init_angle_l, leftRoll);
    [newer_axis_l, newer_angle_l] = rotate_axis_angle_around_local_y(new_axis_l, new_angle_l, leftPitch);
    planeL.setOrientation(AxisAngle ([newer_axis_l, deg2rad(newer_angle_l)]));

    [new_axis_r, new_angle_r] = rotate_axis_angle_around_local_x(init_axis_r, init_angle_r, rightRoll);
    [newer_axis_r, newer_angle_r] = rotate_axis_angle_around_local_y(new_axis_r, new_angle_r, rightPitch);
    planeR.setOrientation(AxisAngle ([newer_axis_r, deg2rad(newer_angle_r)]));

    ah.step();

    pause(1);

    root.createFibulaOptimizationTwo(zOffset);

    % Perform simulation steps
    for i = 1:100
        ah.step();
    end

    % Pause to allow processes to finish
    pause(6);

    % Create screws and export files
    root.getPlatePanel.createScrews();
    
    root.exportFilesTwo();
    root.exportFemPlateTwo();

    
    fileList = {'donor_opt0.obj', 'donor_opt1.obj', 'plate_opt.art', 'resected_mandible_l_opt.obj', 'resected_mandible_r_opt.obj', 'screw_opt0.obj', 'screw_opt1.obj'};

    for i = 1:length(fileList)
        sourceFile = fullfile(sourceDir, fileList{i});
        copyfile(sourceFile, destinationDir);
    end

    % Close the first Artisynth instance
    ah.quit();
    ah = [];
    java.lang.System.gc();

    % Run PyMeshLab remeshing script
    pyrunfile('PymeshlabRemeshTwo.py');
    
    pause(6);

    % Run the second Artisynth model
    try
        ah1 = artisynth('-model', 'artisynth.istar.TMJModel.JawTMJ.JawFemDemoOptimizeTwo');
        if isempty(ah1)
            error('Failed to initialize the second Artisynth instance.');
        end
    catch ME
        disp('Error during second ArtiSynth initialization:');
        disp(ME.message);
        rethrow(ME);
    end
    
     root = ah1.root();

    if defectType == "RB"
        sphm_R = ah1.find ('models/jawmodel/axialSprings/sphm_R');
        sphm_R_parent = sphm_R.getParent();
        sphm_R_parent.remove (sphm_R);

        stm_R = ah1.find ('models/jawmodel/axialSprings/stm_R');
        stm_R_parent = stm_R.getParent();
        stm_R_parent.remove (stm_R);

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
    %pause(3);
    %ah1.quit();
    %ah1 = [];
    %java.lang.System.gc();