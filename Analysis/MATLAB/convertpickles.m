function convertpickles(varargin)
% convert a list of pickles to mat files
% lloyd russell 2016

% select main directory
if ~numel(varargin)
    directory = uigetdir();
else
    directory = varargin{1};
end


% build file list
filelist = glob([directory filesep '**' filesep '*.pkl']);
num_files = numel(filelist);


% convert file list
h = waitbar(0);
for f = 1:num_files
    
    % update progress bar
    waitbar(f/num_files, h, ['Current file: ' strrep(strrep(filelist{f},'\','/'), '_','\_')], 'Interpreter','none');
    
    % do conversion
    pickle2mat(filelist{f}, 'overwrite',false);
    
end
close(h)