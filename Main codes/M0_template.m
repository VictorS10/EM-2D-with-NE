close all;
clear all;
clc
% ---------------------------------------------------------------------------------------------
currentfolder = pwd; % save current path
cd ..                % go one folder down (go out of the current folder and stay in the previous one)
add_paths;           % add all folders where all files are founded in order to acces to them
cd(currentfolder);   % return to the original path
% ---------------------------------------------------------------------------------------------

echo on
% Template
% ==============================
% Creation: 25/oct/2022
% Last modification: --/---/----
% ---------------------------------------
% Main description.
% ---------------------------------------
%    - Some details
%    - Some details
%    - Some details
%    - Some details
echo off

% Parameters
% ============================
T = 0.5; % Time step
S = 0.3; % Step length
clD = 0.15; % Step width





%% End of the code
% ----------------------------------------------------------------------------------------------
cd ..                  % Go down one folder (go out from the current folder and stay in the previous one)
remove_paths;          % Remove the paths of the folders added at the begining
cd(currentfolder);     % Return to the original folder
% ----------------------------------------------------------------------------------------------