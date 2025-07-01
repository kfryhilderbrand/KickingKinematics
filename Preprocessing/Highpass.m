%% Function to apply Highpass filter to data  
%This function will design a highpass filter with order n and cutoff frequency 
%fc.  This filter will then be applied to input data X.

%% Inputs: 
%X:       unfiltered data
%n:       order of the highpass filter
%fc:      cutoff frequency in Hz
%fs:      sampling frequency in Hz
%% Outputs:
%Xfilt:   Filtered Data cell array

function [Xfilt]=Highpass(X, n, fc, fs)

df = designfilt('highpassfir','FilterOrder',n,'CutoffFrequency',fc,...
  'SampleRate',fs);

Xfilt=X;
for i=1:length(X)
    for j=1:length(X{i})
        Xfilt{i}{j} = filtfilt(df,X{i}{j});
    end
end

disp('Done Filtering')
end