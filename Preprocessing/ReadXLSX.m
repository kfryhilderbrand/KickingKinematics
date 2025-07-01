%% Function to read all xlsx data files into matlab array 
% 
%% Inputs:
%filename:  
%% Outputs:
%none
function [] = ReadXLSX(filename)

dinfo = dir('*.xlsx');
names_cell = {dinfo.name};
Truth = cell(1,length(names_cell));
for i=1:length(names_cell)
    Truth{i} = cell(1,2);
    %Truth{i}{1} = xlsread(sprintf('%s',names_cell{i}));
    Truth{i}{1} = readtable(sprintf('%s',names_cell{i}),'ReadVariableNames',false);
    Truth{i}{1} = table2array(Truth{i}{1}(:,:));
    Truth{i}{2} = names_cell{i};

end

%filename = 'Truth.mat';
save(filename)
end