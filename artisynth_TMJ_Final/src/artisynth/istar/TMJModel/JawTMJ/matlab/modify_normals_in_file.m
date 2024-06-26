function modify_normals_in_file(input_filename, output_filename, new_normal1, new_normal3)
    % Define the folder path
    folderPath = 'C:\Users\Hamidreza\git\artisynth_istar\src\artisynth\istar\reconstruction\optimizationBTest';
    
    % Construct the full file paths
    inputFilePath = fullfile(folderPath, input_filename);
    outputFilePath = fullfile(folderPath, output_filename);
    
    % Read the file contents
    fileContents = fileread(inputFilePath);
    
    % Split the contents into lines
    lines = strsplit(fileContents, '\n');
    
    % Modify the first and third lines with the new normals
    lines{1} = sprintf('%f %f %f', new_normal1);
    lines{3} = sprintf('%f %f %f', new_normal3);
    
    % Write the modified lines back to the new file
    fid = fopen(outputFilePath, 'w');
    for i = 1:length(lines)
        fprintf(fid, '%s\n', lines{i});
    end
    fclose(fid);
end
