%% Function to read all csv truth files into matlab array 
%%%  Will read CSV files in current "Truth" directory
%% Inputs:
 
%% Outputs:

function [] = ReadCSV_Truth(~)

dinfo = dir('*.csv');
names_cell = {dinfo.name};
Truth = cell(1,length(names_cell));
for i=1:length(names_cell)
    Truth{i} = cell(1,2);
    Truth{i}{1} = readtable(sprintf('%s',names_cell{i}),'ReadVariableNames',false);
    Truth{i}{1} = table2array(Truth{i}{1}(:,:));
    Truth{i}{2} = names_cell{i};
end

end