function loss = runArtisynthSimTwo(params)
    
    defectType = 'RB'; 
    trial = 4;
    Safety_On = false;

    resultsFile = ['Finl_Result_' defectType '_Defect_Trial_' num2str(trial) '.mat'];
    PercentFile = ['Final_Percent_' defectType '_Defect_Trial_' num2str(trial) '.txt'];
    logFile = ['Final_Log_' defectType '_Defect_Trial_' num2str(trial) '.txt'];
    

    if   Safety_On == true
        safetyFile = ['Safety' defectType '_Defect_Trial_' num2str(trial) '.txt'];
    end


    addpath(fullfile('..','..', '..', '..', '..', '..', '..', '..', 'artisynth_core', 'matlab'));
    setArtisynthClasspath(getenv('ARTISYNTH_HOME'));  
 


    sourceDir = fullfile('..','..','..', '..', '..', '..', '..', '..', 'artisynth_istar', 'src', 'artisynth', 'istar', 'reconstruction', 'optimizationResultTwo');
    destinationDir = fullfile('..', 'geometry');
    %sourceDir = 'C:\Users\Hamidreza\git\artisynth_istar\src\artisynth\istar\reconstruction\optimizationResult';
    %destinationDir = 'C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry';

    bodyList = fullfile('..', 'geometry', 'bodyList.txt');
    %bodyList ="C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry\bodyList.txt";

    toggleComment(bodyList, 'screw1', 'remove');
    toggleComment(bodyList, 'donor_mesh1', 'remove');


    if defectType == "RB"
        
        removeRBMuscles();

    end

    num_screws = 2;
    num_segment = 2;

    zOffset = double(params.zOffset);
    rdpOffset = double(params.rdpOffset);
    leftRoll = double(params.leftRoll);
    leftPitch = double(params.leftPitch);
    rightRoll = double(params.rightRoll);
    rightPitch = double(params.rightPitch);

    % Debugging information
    fprintf('Running simulation with zOffset = %.2f, rdpOffset = %.2f, leftRoll = %.2f, leftPitch = %.2f, rightRoll = %.2f, rightPitch = %.2f\n', ...
        zOffset, rdpOffset, leftRoll, leftPitch, rightRoll, rightPitch);
    fprintf('ARTISYNTH_HOME = %s\n', getenv('ARTISYNTH_HOME'));


    if defectType == "RB"

        % Calculate the new resection plane
    
        % Left Plane
        init_axis_l = [-0.35978 -0.83742 -0.41145];
        init_angle_l = 99.373;
       
    
        % Right Plane
        init_axis_r = [0.74092 -0.52003 0.42498];
        init_angle_r = 149.41;
    

    end

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

    root.importFibulaOptimizationTwo();
    
    import maspack.matrix.AxisAngle;

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

    root.createFibulaOptimizationTwo(zOffset,rdpOffset);

    % Perform simulation steps
    for i = 1:50
        ah.step();
    end

    % Pause to allow processes to finish
    pause(6);

    % Create screws and export files
    root.getPlatePanel.createScrews();
    
    ah.step();  

    root.exportFilesTwo();
    root.exportFemPlateTwo();

    left_mandible = ah.find('models/Reconstruction/rigidBodies/mandibleL');
    right_mandible = ah.find('models/Reconstruction/rigidBodies/mandibleR');


    if left_mandible.getMass() == 0 || right_mandible.getMass() == 0

        disp('Graphic Error...');
        fileID = fopen(logFile, 'a');
        fprintf(fileID, 'Graphic Error:\n');
        fprintf(fileID, '%.2f,', zOffset);
        fprintf(fileID, '%.2f,', rdpOffset);
        fprintf(fileID, '%.2f,', leftRoll);
        fprintf(fileID, '%.2f,', leftPitch);
        fprintf(fileID, '%.2f,', rightRoll);
        fprintf(fileID, '%.2f,', rightPitch);
        fprintf(fileID, '\n');
        fclose(fileID);
        % Use system command to restart MATLAB
        matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
        matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, 'mainScriptModifiedModifiedTwo');
        system(matlabCommand);
        exit; % Close the current MATLAB session

    end
    
    
    fileList = {'donor_opt0.obj', 'donor_opt1.obj', 'plate_opt.art', 'resected_mandible_l_opt.obj', 'resected_mandible_r_opt.obj', 'screw_opt0.obj','screw_opt1.obj'};

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
        ah1 = artisynth('-model', 'artisynth.istar.TMJModel.JawTMJ.JawFemDemoOptimizeTwoWithSafety');
        if isempty(ah1)
            error('Failed to initialize the second Artisynth instance.');
        end
    catch ME
        disp('Error during second ArtiSynth initialization:');
        disp(ME.message);
        rethrow(ME);
    end


     if defectType == "RB"
        sphm_R = ah1.find ('models/jawmodel/axialSprings/sphm_R');
        sphm_R_parent = sphm_R.getParent();
        sphm_R_parent.remove (sphm_R);

        stm_R = ah1.find ('models/jawmodel/axialSprings/stm_R');
        stm_R_parent = stm_R.getParent();
        stm_R_parent.remove (stm_R);

    end
    
    for i = 1:1240
        ah1.step();
    end

    left_percent = ah1.getOprobeData('5');
    right_percent = ah1.getOprobeData('6');

    left_safety = ah1.getOprobeData('7');
    right_safety = ah1.getOprobeData('8');

    mid0_percent = ah1.getOprobeData('9');
    mid1_percent = ah1.getOprobeData('10');

    if length(left_percent) < 2 || length(right_percent) < 2
        disp('Biomedical Error...');
        fileID = fopen(logFile, 'a');
        fprintf(fileID, 'Biomedical Error:\n');
        fprintf(fileID, '%.2f,', zOffset);
        fprintf(fileID, '%.2f,', rdpOffset);
        fprintf(fileID, '%.2f,', leftRoll);
        fprintf(fileID, '%.2f,', leftPitch);
        fprintf(fileID, '%.2f,', rightRoll);
        fprintf(fileID, '%.2f,', rightPitch);
        fprintf(fileID, '\n');
        fclose(fileID);
        % Use system command to restart MATLAB
        matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
        matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, 'mainScriptModifiedModifiedTwo');
        system(matlabCommand);
        exit; % Close the current MATLAB session
    end

    % Calculate the loss

    loss1 = - (0.5*(mean(left_percent(:,2)) + min(mean(mid0_percent(:,2)),mean(mid1_percent(:,2)))) - 0.499 *abs(mean(left_percent(:,2)) - min(mean(mid0_percent(:,2)),mean(mid1_percent(:,2)))));


     if loss1 == 0.00 || isnan (loss1)
        disp('Zeros/Nan loss...');
        fileID = fopen(logFile, 'a');
        fprintf(fileID, 'Zero/nan Loss Error:\n');
        fprintf(fileID, '%.4f,', zOffset);
        fprintf(fileID, '%.2f,', rdpOffset);
        fprintf(fileID, '%.4f,', leftRoll);
        fprintf(fileID, '%.4f,', leftPitch);
        fprintf(fileID, '%.4f,', rightRoll);
        fprintf(fileID, '%.4f,', rightPitch);
        fprintf(fileID, '\n');
        fclose(fileID);
        % Use system command to restart MATLAB
        matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
        matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, 'mainScriptModifiedModified');
        system(matlabCommand);
        exit; % Close the current MATLAB session
    end


     if Safety_On == true
        loss2 =  calculateSafetyFactorsCost(left_safety, right_safety);
    else
        loss2 = 0 ;
    end
    
    loss = loss1 + loss2 ; 

    % Append left and right percent to a text file
    fileID = fopen(PercentFile, 'a');
    fprintf(fileID, 'Left Percent:\n');
    fprintf(fileID, '%.2f,', left_percent(:,2));
    fprintf(fileID, '\nRight Percent:\n');
    fprintf(fileID, '%.2f,', right_percent(:,2));
    fprintf(fileID, '\nMid0 percent:\n');
    fprintf(fileID, '%.2f,', mid0_percent(:,2));
     fprintf(fileID, '\nMid1 percent:\n');
    fprintf(fileID, '%.2f,', mid1_percent(:,2));
    fprintf(fileID, '\n');
    fclose(fileID);

    if Safety_On == true
        fileID = fopen(safetyFile, 'a');
        fprintf(fileID, 'Left Safety:\n');
        fprintf(fileID, '%.2f,', left_safety(:,2));
        fprintf(fileID, '\nRight Safety:\n');
        fprintf(fileID, '%.2f,', right_safety(:,2));
        fprintf(fileID, '\n');
        fclose(fileID);
    end


    % Close the second Arisynth instance
    pause(3);
    ah1.quit();
    ah1 = [];
    java.lang.System.gc();
end
