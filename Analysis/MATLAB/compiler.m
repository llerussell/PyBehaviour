%% PyBehaviour: Compiler (MATLAB)
% Lloyd Russell

%%
subjects = {''};

%%
current_mfile_path = mfilename('fullpath');
[current_mfile_dirname, current_mfile_filename] = fileparts(current_mfile_path);
cd(current_mfile_dirname)
cd('../../')
try
    cd('Results')
catch
    warning('Results directory does not exist')
end

%% compile
num_subjects = length(subjects);
results = [];
for m = 1:num_subjects
    current_subject = subjects{m};
    results.(current_subject) = [];
end

%% analyse


%% plot