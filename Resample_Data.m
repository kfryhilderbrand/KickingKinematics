%% Function to resample the sensor data to a uniform time vector. 
%%%Will create an overall time vector using the start and end time found in
%%%StartEndTime function. Will resample the sensor data to those timestamps.
%% Inputs:
%XRaw: 	    unfiltered data. Cell array where each cell represents data from
%           a different sensor (and each cell could have multiple
%           measurements like Accel and Gyro)
%fs:        sampling frequency
%starttime: maximum start time of all sensor data and the start time of the
%           overall time vector.
%endtime:   minimum end time of all sensor data and the end time of the
%           overall time vector.

%% Outputs:
%XResamp: Resampled data
%XResampDemean: Resampled data with demeaned data channels

function [XResamp, XResampDemean] = Resample_Data(XRaw,fs, starttime, endtime)
XResamp = cell(1,length(XRaw));
XResampDemean = cell(1,length(XRaw));

%% Resamples to overall time vector
% Overall time vector
time = starttime:1/fs:endtime;

% Resamples each sensor (i.e. each XRaw{i}) and each component (i.e. each
% cell within XRaw{i}) to universal time vector
for i=1:length(XRaw)
    XResamp{i} = cell(1,length(XRaw{i}));
    XResampDemean{i} = cell(1,length(XRaw{i}));

    for j=1:length(XRaw{i})
        [~,width]=size(XRaw{i}{j});
        temp=interp1(XRaw{i}{j}(:,1),XRaw{i}{j}(:,2:width),time,'pchip',0); %interpolates to time vector as defined by truth
        %note:will default to
        %0 for values outside
        %of truth vector
        XResamp{i}{j}=[time', temp];
        
        for k=2:width
            %%% Demean channels to account for channel bias
            XRaw{i}{j}(:,k) = XRaw{i}{j}(:,k)-mean(XRaw{i}{j}(:,k));
        end
        
        temp=interp1(XRaw{i}{j}(:,1),XRaw{i}{j}(:,2:width),time,'pchip',0); %interpolates to time vector as defined by truth
        %note:will default to
        %0 for values outside
        %of truth vector
        XResampDemean{i}{j}=[time', temp];
    end
    
end
