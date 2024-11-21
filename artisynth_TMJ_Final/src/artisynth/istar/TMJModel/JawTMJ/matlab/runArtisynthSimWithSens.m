function loss = runArtisynthSimWithSens(params)
    
    defectType = 'B'; 
    trial = 1;

    Safety_On = false;

    resultsFile = ['Sens_Finl_Result_Sensitivity_' defectType '_Defect_Trial_' num2str(trial) '.mat'];
    PercentFile = ['Sens_Final_Percent_Sensitivity_' defectType '_Defect_Trial_' num2str(trial) '.txt'];
    logFile = ['Sens_Final_Log_Sensitivity_' defectType '_Defect_Trial_' num2str(trial) '.txt'];


    if   Safety_On == true
        safetyFile = ['Safety_Sensitivity_' defectType '_Defect_Trial_' num2str(trial) '.txt'];
    end


    iterationFile = 'currentIteration.mat';

    % Load or initialize current iteration from file
    if isfile(iterationFile)
        load(iterationFile, 'currentIteration');  % Load iteration if the file exists
    else
        currentIteration = 1;  % Initialize if the file does not exist
    end

    % Increment current itera

    addpath(fullfile('..','..', '..', '..', '..', '..', '..', '..', 'artisynth_core', 'matlab'));
    setArtisynthClasspath(getenv('ARTISYNTH_HOME'));  


    %sourceDir = 'C:\Users\Hamidreza\git\artisynth_istar\src\artisynth\istar\reconstruction\optimizationResult';
    %destinationDir = 'C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry';

    bodyList = fullfile('..', 'geometry', 'bodyList.txt');
    %bodyList ="C:\Users\Hamidreza\git\artisynth_tmj_final\artisynth_TMJ_Final\src\artisynth\istar\TMJModel\JawTMJ\geometry\bodyList.txt";
    
    toggleComment(bodyList, 'screw1', 'add');
    toggleComment(bodyList, 'donor_mesh1', 'add');


    resetMuscles();

    if defectType == "B"

        removeBMuscles();

    elseif defectType == "S"

        removeSMuscles();
    end
  


    numParameters = 11;
    
    % Determine parameter index (1 to 11) based on current iteration
    paramIdx = mod(ceil(currentIteration / 8) - 1, numParameters) + 1
    
    % Determine adjustment factor: 0.9 for the first 4 iterations, 1.1 for the next 4
    adjustment = (mod(currentIteration - 1, 8) < 4) * 0.9 + (mod(currentIteration - 1, 8) >= 4) * 1.1


    % Run the second Artisynth model
    try
        ah1 = artisynth();
        ah1.loadModel('artisynth.istar.TMJModel.JawTMJ.JawFemDemoOptimizeWithSens', num2str(paramIdx), num2str(adjustment))
        if isempty(ah1)
            error('Failed to initialize the second Artisynth instance.');
        end
    catch ME
        disp('Error during second ArtiSynth initialization:');
        disp(ME.message);
        rethrow(ME);
    end


   if paramIdx == 10
       
         % List of muscles (using 'r' as the base, will replace with 'l' for left side)
        muscles = {'rat', 'rmt', 'rpt', 'rip', 'rdm', 'rsm', 'rmp'};
        
        % Loop through each muscle, adjusting both right and left sides
        for i = 1:length(muscles)
            % Right-side muscle
            muscle_name_right = ['models/jawmodel/axialSprings/', muscles{i}];
            muscle_right = ah1.find(muscle_name_right);
            material_right = muscle_right.getMaterial();
            material_right.setOptLength(material_right.getOptLength() * adjustment);
        
            % Left-side muscle (replace 'r' with 'l')
            muscle_name_left = strrep(muscle_name_right, '/r', '/l');
            muscle_left = ah1.find(muscle_name_left);
            material_left = muscle_left.getMaterial();
            material_left.setOptLength(material_left.getOptLength() * adjustment);
        end

   end 

    if paramIdx == 11
         % List of muscles (using 'r' as the base, will replace with 'l' for left side)
        muscles = {'rat', 'rmt', 'rpt', 'rip', 'rdm', 'rsm', 'rmp'};
        
        % Loop through each muscle, adjusting both right and left sides
        for i = 1:length(muscles)
            % Right-side muscle
            muscle_name_right = ['models/jawmodel/axialSprings/', muscles{i}];
            muscle_right = ah1.find(muscle_name_right);
            material_right = muscle_right.getMaterial();
            material_right.setMaxForce(material_right.getMaxForce() * adjustment);
        
            % Left-side muscle (replace 'r' with 'l')
            muscle_name_left = strrep(muscle_name_right, '/r', '/l');
            muscle_left = ah1.find(muscle_name_left);
            material_left = muscle_left.getMaterial();
            material_left.setMaxForce(material_left.getMaxForce() * adjustment);
        end 

    end
    
    for i = 1:1240
        ah1.step();
    end

    left_percent = ah1.getOprobeData('5');
    right_percent = ah1.getOprobeData('6');
    
    left_safety = ah1.getOprobeData('7');
    right_safety = ah1.getOprobeData('8');


    if length(left_percent) < 2 || length(right_percent) < 2
        disp('Biomedical Error...');
        fileID = fopen(logFile, 'a');
        fprintf(fileID, 'Biomedical Error:\n');
        fprintf(fileID, '%.2f,', zOffset);
        fprintf(fileID, '%.2f,', leftRoll);
        fprintf(fileID, '%.2f,', leftPitch);
        fprintf(fileID, '%.2f,', rightRoll);
        fprintf(fileID, '%.2f,', rightPitch);
        fprintf(fileID, '\n');
        fclose(fileID);
        % Use system command to restart MATLAB
        matlabExecutable = fullfile(matlabroot, 'bin', 'matlab');
        matlabCommand = sprintf('"%s" -r "load(''%s''); run(''%s''); exit"', matlabExecutable, resultsFile, 'mainScriptModifiedModified');
        system(matlabCommand);
        exit; % Close the current MATLAB session
    end


    loss1 = - (0.5*(mean(left_percent(:,2)) + mean(right_percent(:,2))) - 0.499 *abs(mean(left_percent(:,2)) - mean(right_percent(:,2)))) + .0001;
   
    if loss1 == 0.00 || isnan (loss1)
        disp('Zeros/Nan loss...');
        fileID = fopen(logFile, 'a');
        fprintf(fileID, 'Zero/nan Loss Error:\n');
        fprintf(fileID, '%.4f,', zOffset);
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
    fprintf(fileID, '\n');
    fclose(fileID);

    if Safety_On == true
        fileID = fopen(safetyFile, 'a');
        fprintf(fileID, 'Left Percent:\n');
        fprintf(fileID, '%.2f,', left_safety(:,2));
        fprintf(fileID, '\nRight Percent:\n');
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
