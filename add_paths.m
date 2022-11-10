% "add_paths.m"
% ----------------------------
% Add the paths where the files are located
% All the files are located in folders. We add the folders in order to use those files.
% In this case we are gonna add the files inside of this folder (by calling
% this code in any of the codes in the main folder
% If you want to create another folder to organize your files, just do it.

address = pwd; % Current path
if isunix
    slash = '/';
else
    slash = '\';
end

FilesAddress = genpath(address);
% Adding main folder and all subfolders in there
addpath(FilesAddress);
% ----------------------------------------

fprintf('Paths for all folders in %s, were added =)|\n',address);

% With the instruction "path" we can see in the command display all the paths that are currently added in MATLAB
% ----------------------------------------------------------------------------------------------------------------------
% IMPORTANT NOTE: MATLAB use these paths to go inside the folders and run the files found there. However, if MATLAB 
%                 has access to other files with the same name (in other folders), we will not be sure which file will
%                 be taken into account by MATLAB, so there will be a mess. That is why it is strongly recommended to
%                 remove the paths we just added when they are not used anymore. In order to ensure this, I always
%                 remove the paths added at the end of any main file by using the "remove_paths.m" file
