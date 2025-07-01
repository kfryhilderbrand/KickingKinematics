%% Function to apply moving average filter to data  
%This function will design a moving average filter with width n.
%This filter will then be applied to input data X.

%% Inputs: 
%X:       unfiltered data
%n:       length of moving avg window

%% Outputs:
%Xfilt:   Filtered Data cell array

function [Xfilt]=MovAvg(X, n)

Xfilt=X;
for i=1:length(X)
    for j=1:length(X{i})
        Xfilt{i}{j} = [Xfilt{i}{j}(:,1),movmean(X{i}{j}(:,2),n),movmean(X{i}{j}(:,3),n),movmean(X{i}{j}(:,4),n)];
    end
end
 
end