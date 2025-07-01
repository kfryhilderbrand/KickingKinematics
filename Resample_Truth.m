%% Function to resample data
%This function will interpolate the truth vector to a uniform time vector
%as determined by the cell array of Labels (L) of size NumMeas x NumSensors.
%% Inputs:
%%%     Truth: vector of truth labels
%%%     fs: sampling freq.
%% Outputs:
%%%     Truth: truth before resampling
%%%     TruthResamp: resampled truth

function [TruthData,TruthResamp] = Resample_Truth(fs,Truth)

step = str2double(sprintf('%0.2f',1/fs));
TruthResamp=cell(1,length(Truth));
TruthData=cell(1,length(Truth));

timestart = str2double(sprintf('%0.2f',Truth{1}{1}(1,1)));   %round to nearest hundreth
timeend = str2double(sprintf('%0.2f',Truth{1}{1}(end,1)));   %round to nearest hundreth
time = (timestart:step:timeend)';

for i=1:length(Truth)
    [Uniq_Time, indx, ~] = unique(Truth{i}{1}(:,1));
    Uniq_Vals = Truth{i}{1}(indx,2);
    temp = interp1(Uniq_Time,Uniq_Vals,time,'pchip');
    TruthResamp{i}=[time, temp];
    TruthData{i}=Truth{i}{1};
    
    %Take care of edge cases (i.e. when Truth is between values)
    TruthResamp{i}(:,2) = round(TruthResamp{i}(:,2),0);
end

end
