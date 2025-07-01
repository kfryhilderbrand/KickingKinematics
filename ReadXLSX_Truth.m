%% Function to read all xlsx truth files into matlab array 
%  Will read XLSX files in current "Truth" directory
%% Inputs:
 
%% Outputs:

function [] = ReadXLSX_Truth(~)

dinfo = dir('*.xlsx');
names_cell = {dinfo.name};
Truth = cell(1,length(names_cell));
for i=1:length(names_cell)
    Truth{i} = cell(1,2);
    Truth{i}{1} = xlsread(sprintf('%s',names_cell{i}));
    Truth{i}{2} = names_cell{i};
end

end