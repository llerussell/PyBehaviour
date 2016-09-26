function varargout = pickle2mat(varargin)
% This function will convert a serialised python object (a pickle) into 
% a MATLAB .mat file. It calls python commands from MATLAB, to first
% load the pickle, then resave it using scipy's savemat. If the .mat file
% already exists, this function will only reconvert and overwrite if told to.
% 
% Input
% -----
% filename - str 
%     optional, must have .pkl as extension
% overwrite - bool
%      optional, 
% 
% Output
% ------
% A .mat file will be saved out in the same directory
% If requested a MATLAB struct will be returned containing the converted
% data
%
% Notes
% -----
% Calling python commands from MATLAB was introduced in version 2014b
% inputParser was introduced in MATLAB 2007a
% 
% Author
% ------
% Lloyd Russell 2016 @llerussell


% parse inputs
p = inputParser;
p.addOptional('filename', '');
p.addOptional('overwrite', false);
parse(p, varargin{:});
filename = p.Results.filename;
overwrite = p.Results.overwrite;

% open file select dialog if no filename provided
if strcmpi(filename, '')
   [filename,pathname] = uigetfile('*.pkl'); 
   filename = [pathname filesep filename];
end

% generate .mat filename
matname = strrep(filename, '.pkl', '.mat');

% convert pickle to mat
if ~exist(matname, 'file') || (exist(matname, 'file') && overwrite)
    % call python script to deserialise pickle and resave as mat file
    py.scipy.io.savemat(matname, py.pickle.load(py.open(filename, 'rb'), pyargs('encoding','latin1')));
end

% load .mat file, if requested
if (nargout) || strcmpi(p.Results.filename,'')
    varargout{1} = load(matname);
end
