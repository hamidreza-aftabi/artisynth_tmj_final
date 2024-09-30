    
    
    clc
    clear all
    defectType = "S"; 


    %addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
    %setArtisynthClasspath(getenv('ARTISYNTH_HOME'));

    addpath(fullfile('..','..', '..', '..', '..', '..', '..', '..', 'artisynth_core', 'matlab'));
    setArtisynthClasspath(getenv('ARTISYNTH_HOME'));  


    %sourceDir = 'C:\Users\Hamidreza\git\artisynth_istar\src\artisynth\istar\reconstruction\optimizationResult';
    %destinationDir = 'C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry';
    sourceDir = fullfile('..','..','..', '..', '..', '..', '..', '..', 'artisynth_istar', 'src', 'artisynth', 'istar', 'reconstruction', 'optimizationResult');
    destinationDir = fullfile('..', 'geometry');

    %bodyList ="C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry\bodyList.txt";
    bodyList = fullfile('..', 'geometry', 'bodyList.txt');
   
    toggleComment(bodyList, 'screw1', 'add');
 
    resetMuscles();

    if defectType == "S"

        removeSMuscles();

    end


    num_screws = 1;
    num_segment = 1;
    Screw_lenght = 35;

    %zOffset = double(params.zOffset);
    %leftRoll = double(params.leftRoll);
    %leftPitch = double(params.leftPitch);
    %rightRoll = double(params.rightRoll);
    %rightPitch = double(params.rightPitch);

    %zOffset = -7;
    %leftRoll =  0;
    %leftPitch = 0;
    %rightRoll =  0;
    %rightPitch = 0;

 
    %zOffset = 3;
    %leftRoll =  10;
    %leftPitch = -15;
    %rightRoll =  -10;
    %rightPitch = -15;

    zOffset = 1.795;
    leftRoll =  -5.33;
    leftPitch = 3.33;
    rightRoll =  9.65;
    rightPitch = 14.77;



    % Debugging information
    fprintf('Running simulation with zOffset = %.2f, leftRoll = %.2f, leftPitch = %.2f, rightRoll = %.2f, rightPitch = %.2f\n', ...
        zOffset, leftRoll, leftPitch, rightRoll, rightPitch);
    fprintf('ARTISYNTH_HOME = %s\n', getenv('ARTISYNTH_HOME'));


    
    % Calculate the new resection plane
    % Left Plane
    
    %init_axis_l =[ 0.30742 -0.93415 0.18128];
    %init_angle_l = 105.46;
    init_axis_l = [0.30742 -0.93415 0.18128];
    init_angle_l =  105.46;

    % Right Plane
    %init_axis_r = [0.73745 -0.29537 0.60739];
    %init_angle_r =  157.51;
    init_axis_r = [0.72239 -0.29051 0.6275]; 
    init_angle_r = 156.63;

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
   
    root.getPlateBuilder().setScrewLength (Screw_lenght)
    root.getPlateBuilder().setNumScrews (num_screws);
    root.getSegmentGenerator.setMaxSegments(num_segment);
    root.getSegmentGenerator.setNumSegments (num_segment);

    root.importFibulaOptimizationS();
    
    import maspack.matrix.AxisAngle ;

    planeL = ah.find('models/Reconstruction/resectionPlanes/planeL');
    planeL.setOrientation(AxisAngle ([init_axis_l, deg2rad(init_angle_l)]));

    planeR = ah.find('models/Reconstruction/resectionPlanes/planeR');
    planeR.setOrientation(AxisAngle ([init_axis_r, deg2rad(init_angle_r)]));

    ah.step();
    
    pause(5);

    [new_axis_l, new_angle_l] = rotate_axis_angle_around_local_x(init_axis_l, init_angle_l, leftRoll);
    [newer_axis_l, newer_angle_l] = rotate_axis_angle_around_local_y(new_axis_l, new_angle_l, leftPitch);
    planeL.setOrientation(AxisAngle ([newer_axis_l, deg2rad(newer_angle_l)]));

    [new_axis_r, new_angle_r] = rotate_axis_angle_around_local_x(init_axis_r, init_angle_r, rightRoll);
    [newer_axis_r, newer_angle_r] = rotate_axis_angle_around_local_y(new_axis_r, new_angle_r, rightPitch);
    planeR.setOrientation(AxisAngle ([newer_axis_r, deg2rad(newer_angle_r)]));



    ah.step();

     