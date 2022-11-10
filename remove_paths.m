% "remove_paths.m"
% ----------------------------
% Remove the paths added in "add_paths.m".
address = pwd;
if isunix
    slash = '/';
else
    slash = '\';
end

FilesAddress = genpath(address);
% Removing main folder and all subfolders in there
rmpath(FilesAddress);
% ----------------------------------------
fprintf('Paths for all folders in %s were removed =(\n',address);