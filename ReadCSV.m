%% Function to read all CSV data files into matlab array
%%% Will read CSV files in current "Data" directory
%% Inputs:
%%% filename: name of file to save Data to
%%% HEADERflag: 1 if csv files have a header, 0 if not
%% Outputs:

function [] = ReadCSV(filename, HEADERflag)

dinfo=dir('*.csv');
names_cell = {dinfo.name};
Data = cell(1,length(names_cell));

for i=1:length(names_cell)
    Data{i} = cell(1,2);
    if HEADERflag
        data = readtable(sprintf('%s',names_cell{i}),'ReadVariableNames',false);
        data = table2array(data(2:end,2:end));
        if iscell(data)
            if ischar(data{1})
                temp = cellfun(@str2num,data,'un',0);
            end
        Data{i}{1} = cell2mat(temp); %header and first column not included
        end
        %%% To convert data stored as chars in cells into a numerical array
        Data{i}{2} = names_cell{i};
    else
        Data{i}{1} = readtable(sprintf('%s',names_cell{i}),'ReadVariableNames',false);
        Data{i}{1} = table2array(Data{i}{1}(:,2:end));
        Data{i}{2} = names_cell{i};
    end
end

save(filename, 'Data')
end