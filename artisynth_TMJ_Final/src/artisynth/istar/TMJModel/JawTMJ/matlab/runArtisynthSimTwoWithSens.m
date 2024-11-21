function loss = runArtisynthSimTwoWithSens(params)
    
    defectType = 'RB'; 
    trial = 4;
    Safety_On = false;

    resultsFile = ['Finl_Result_' defectType '_Defect_Trial_' num2str(trial) '.mat'];
    PercentFile = ['Final_Percent_' defectType '_Defect_Trial_' num2str(trial) '.txt'];
    logFile = ['Final_Log_' defectType '_Defect_Trial_' num2str(trial) '.txt'];
    

    if   Safety_On == true
        safetyFile = ['Safety' defectType '_Defect_Trial_' num2str(trial) '.txt'];
    end

   iterationFile = 'currentIteration.mat';

    % Load or initialize current iteration from file
    if isfile(iterationFile)
        load(iterationFile, 'currentIteration');  % Load iteration if the file exists
    else
        currentIteration = 1;  % Initialize if the file does not exist
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

        numParameters = 11;
    
    % Determine parameter index (1 to 11) based on current iteration
    paramIdx = mod(ceil(currentIteration / 8) - 1, numParameters) + 1
    
    % Determine adjustment factor: 0.9 for the first 4 iterations, 1.1 for the next 4
    adjustment = (mod(currentIteration - 1, 8) < 4) * 0.9 + (mod(currentIteration - 1, 8) >= 4) * 1.1




    % Run the second Artisynth model
    try
        ah1 = artisynth();
        ah1.loadModel('artisynth.istar.TMJModel.JawTMJ.JawFemDemoOptimizeTwoWithSafetyWithSens', num2str(paramIdx), num2str(adjustment))
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

    if paramIdx == 10
       
         % List of muscles (using 'r' as the base, will replace with 'l' for left side)
        muscles = {'lat', 'lmt', 'lpt', 'lip', 'ldm', 'lsm', 'lmp', 'rip'};
        
        % Loop through each muscle, adjusting both right and left sides
        for i = 1:length(muscles)
            % Right-side muscle
            muscle_name_right = ['models/jawmodel/axialSprings/', muscles{i}];
            muscle_right = ah1.find(muscle_name_right);
            material_right = muscle_right.getMaterial();
            material_right.setOptLength(material_right.getOptLength() * adjustment);

        end

   end 

    if paramIdx == 11
         % List of muscles (using 'r' as the base, will replace with 'l' for left side)
        muscles = {'lat', 'lmt', 'lpt', 'lip', 'ldm', 'lsm', 'lmp', 'rip'};
        
        % Loop through each muscle, adjusting both right and left sides
        for i = 1:length(muscles)
            % Right-side muscle
            muscle_name_right = ['models/jawmodel/axialSprings/', muscles{i}];
            muscle_right = ah1.find(muscle_name_right);
            material_right = muscle_right.getMaterial();
            material_right.setMaxForce(material_right.getMaxForce() * adjustment);

        end 

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
  
    % Increment current iteration
    currentIteration = currentIteration + 1;
    
    % Save the updated iteration count back to the file
    save(iterationFile, 'currentIteration');

    % Close the second Arisynth instance
    pause(3);
    ah1.quit();
    ah1 = [];
    java.lang.System.gc();
end
