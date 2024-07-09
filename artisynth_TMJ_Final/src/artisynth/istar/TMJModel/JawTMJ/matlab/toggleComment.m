function toggleComment(filename, targetLineStart, action)
    % filename: name of the text file
    % targetLineStart: the start of the target line, e.g., 'screw1'
    % action: 'add' to add a # or 'remove' to remove a #
    
    % Read the file contents
    fid = fopen(filename, 'r');
    fileContents = textscan(fid, '%s', 'Delimiter', '\n');
    fclose(fid);
    lines = fileContents{1};
    
    % Process each line
    for i = 1:length(lines)
        line = lines{i};
        if startsWith(strtrim(line), targetLineStart)
            if strcmp(action, 'add') && ~startsWith(line, '#')
                lines{i} = ['#' line]; % Add the # at the beginning
            elseif strcmp(action, 'remove') && startsWith(line, '#')
                lines{i} = line(2:end); % Remove the # at the beginning
            end
        end
    end
    
    % Write the modified contents back to the file
    fid = fopen(filename, 'w');
    for i = 1:length(lines)
        fprintf(fid, '%s\n', lines{i});
    end
    fclose(fid);
    
    disp('File updated successfully.');
end
