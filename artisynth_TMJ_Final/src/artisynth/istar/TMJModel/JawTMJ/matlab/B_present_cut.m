    
    defectType = "B"; 
    Safety_On = true;

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

    if defectType == "B"

        removeBMuscles();

    end

    num_screws = 1;
    num_segment = 1;

    %zOffset = double(params.zOffset);
    %leftRoll = double(params.leftRoll);
    %leftPitch = double(params.leftPitch);
    %rightRoll = double(params.rightRoll);
    %rightPitch = double(params.rightPitch);
  

    %zOffset = -2;
    %leftRoll =  0;
    %leftPitch = 0;
    %rightRoll =  0;
    %rightPitch = 0;


    %zOffset = -2;
    %leftRoll =  25;
    %leftPitch = 25;
    %rightRoll =  20;
    %rightPitch = 20;


    zOffset = 2.58617968225706;
    leftRoll =  24.7627438650634;
    leftPitch = 23.5412504434493;
    rightRoll =  19.9614740650165;
    rightPitch = 11.3797697972841;

    
    % Debugging information
    fprintf('Running simulation with zOffset = %.2f, leftRoll = %.2f, leftPitch = %.2f, rightRoll = %.2f, rightPitch = %.2f\n', ...
        zOffset, leftRoll, leftPitch, rightRoll, rightPitch);
    fprintf('ARTISYNTH_HOME = %s\n', getenv('ARTISYNTH_HOME'));

    % Calculate the new resection plane

    % Left Plane
    init_axis_l = [-0.35978 -0.83742 -0.41145];
    init_angle_l = 99.373;


    % Right Plane
    init_axis_r = [0.67407 -0.37913 0.63395];
    init_angle_r = 144.41;
   

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
    
    pause(10);

    [new_axis_l, new_angle_l] = rotate_axis_angle_around_local_x(init_axis_l, init_angle_l, leftRoll);
    [newer_axis_l, newer_angle_l] = rotate_axis_angle_around_local_y(new_axis_l, new_angle_l, leftPitch);
    planeL.setOrientation(AxisAngle ([newer_axis_l, deg2rad(newer_angle_l)]));

    [new_axis_r, new_angle_r] = rotate_axis_angle_around_local_x(init_axis_r, init_angle_r, rightRoll);
    [newer_axis_r, newer_angle_r] = rotate_axis_angle_around_local_y(new_axis_r, new_angle_r, rightPitch);
    planeR.setOrientation(AxisAngle ([newer_axis_r, deg2rad(newer_angle_r)]));

    ah.step();
