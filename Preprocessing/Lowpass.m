%% Function to apply Lowpass filter to data  
%This function will design a lowpass filter with order n and cutoff frequency 
%fc.  This filter will then be applied to input data X.

%% Inputs: 
%X:       unfiltered data
%n:       order of the lowpass filter
%fc:      cutoff frequency in Hz
%fs:      sampling frequency in Hz
%% Outputs:
%Xfilt:   Filtered Data cell array

function [Xfilt]=Lowpass(X, n, fc, fs)

df = designfilt('lowpassfir','FilterOrder',n,'CutoffFrequency',fc,...
  'SampleRate',fs);%,'DesignMethod','butter');

Xfilt=X;
for i=1:length(X)
    for j=1:length(X{i})
        Xfilt{i}{j} = filtfilt(df,X{i}{j});
    end
end

disp('Done Filtering')
% 
% disp('Plot Gyroscope')
% close all
%  for i=1:length(Xfilt)
%      figure()
%      subplot(3,1,1)
%      time1 = X{i}{2}(:,1)-X{i}{2}(1,1);
%      time2 = Xfilt{i}{2}(:,1)-X{i}{2}(1,1);
%      plot(time1,X{i}{2}(:,2),time2,Xfilt{i}{2}(:,2));
%  
%      subplot(3,1,2)
%      plot(time1,X{i}{2}(:,3),time2,Xfilt{i}{2}(:,3));
%       
%      subplot(3,1,3)
%      plot(time1,X{i}{2}(:,4),time2,Xfilt{i}{2}(:,4));
%  
%  end
 
end