    
  defectType = "B"; 


    %addpath('C:\Users\Hamidreza\git\artisynth_core\matlab');
    %setArtisynthClasspath(getenv('ARTISYNTH_HOME'));
    addpath(fullfile('..','..', '..', '..', '..', '..', '..', '..', 'artisynth_core', 'matlab'));
    setArtisynthClasspath(getenv('ARTISYNTH_HOME'));    


    %bodyList ="C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry\bodyList.txt";
    bodyList = fullfile('..', 'geometry', 'bodyList.txt');
   
    toggleComment(bodyList, 'screw1', 'add');
 
    resetMuscles();

    if defectType == "B"

        removeBMuscles();

    end

    num_screws = 1;
    num_segment = 1;   



    init_axis_l = [-0.35978 -0.83742 -0.41145];
    init_angle_l = 99.373;

    init_axis_r = [0.67407 -0.37913 0.63395];
    init_angle_r = 144.41;


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
            root.importFibulaOptimization();


    for i=1:50
    


        zOffset = table2array(results.XTrace(i,1));
        leftRoll = table2array(results.XTrace(i,2));
        leftPitch = table2array(results.XTrace(i,3));
        rightRoll = table2array(results.XTrace(i,4));
        rightPitch = table2array(results.XTrace(i,5));
    
        planeL = ah.find('models/Reconstruction/resectionPlanes/planeL');
        planeL.setOrientation(AxisAngle ([init_axis_l, deg2rad(init_angle_l)]));
    
        planeR = ah.find('models/Reconstruction/resectionPlanes/planeR');
        planeR.setOrientation(AxisAngle ([init_axis_r, deg2rad(init_angle_r)]));
    
        ah.step();
        
        %pause(1);
    
        [new_axis_l, new_angle_l] = rotate_axis_angle_around_local_x(init_axis_l, init_angle_l, leftRoll);
        [newer_axis_l, newer_angle_l] = rotate_axis_angle_around_local_y(new_axis_l, new_angle_l, leftPitch);
        planeL.setOrientation(AxisAngle ([newer_axis_l, deg2rad(newer_angle_l)]));
    
        [new_axis_r, new_angle_r] = rotate_axis_angle_around_local_x(init_axis_r, init_angle_r, rightRoll);
        [newer_axis_r, newer_angle_r] = rotate_axis_angle_around_local_y(new_axis_r, new_angle_r, rightPitch);
        planeR.setOrientation(AxisAngle ([newer_axis_r, deg2rad(newer_angle_r)]));
    
        ah.step();
    
        pause(1);
    
        %root.createFibulaOptimization(zOffset);

        %for j=1:100
        %    ah.step()
        %end
    
        % Pause to allow processes to finish
        pause(1);


   end
   