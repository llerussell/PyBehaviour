function data = loadpickle(filename)

% This function will load a serialised python pickle object into the MATLAB
% workspace. It does this by converting the pickle object into a MATLAB
% compatible file. It calls python commands from MATLAB, to first load the
% pickle, then resave it using scipy's savemat. If the .mat file already
% exists, this function will simply load that.
% 
% Input
% -----
% filename - str 
%     must have .pkl as extension
% 
% Output
% ------
% data - struct
%     the converted data
% 
% Notes
% -----
% A .mat file will be saved out in the same directory
% Calling python commands from MATLAB was introduced in verison 2014b
% 
% Alternative:
% % directly load python pickle - but will have to recursively convert all datatypes
% results = struct(py.pickle.load(py.open(fname,'rb')));
% 
% Author
% ------
% Lloyd Russell 2016


% generate the .mat filename
matname = strrep(filename, '.pkl', '.mat');

% if .mat file doesn't exist, create it
if ~exist(matname, 'file')
    % call python script to deserialise pickle and resave as mat file
    py.scipy.io.savemat(matname, ...
                        py.pickle.load(py.open(filename, 'rb'), ...
                                       pyargs('encoding','latin1')));
end

% load in .mat file
data = load(matname);
