function modify_normals_in_file(input_filename, output_filename, new_normal1, new_normal3)
    % Read the file contents
    fileContents = fileread(input_filename);
    
    % Split the contents into lines
    lines = strsplit(fileContents, '\n');
    
    % Modify the first and third lines with the new normals
    lines{1} = sprintf('%f %f %f', new_normal1);
    lines{3} = sprintf('%f %f %f', new_normal3);
    
    % Write the modified lines back to the new file
    fid = fopen(output_filename, 'w');
    for i = 1:length(lines)
        fprintf(fid, '%s\n', lines{i});
    end
    fclose(fid);
end
