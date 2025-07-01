%% Function to Parse through Acceleration Data
%%% Function used to parse through  non-demeaned accelerometer data to 
%%% create windows of data. This data will be used to determine the 
%%% direction of gravity during periods of rest (for joint axis and joint 
%%% rate determinations).
%% Inputs:
%%% X:           Acceleration data to parse

%% Outputs:
%%% SensData:    Struct containing window data and timestamps for center of
%%%              windows

function [SensData]=ParseAccelData(X, windowsize, windowslide)

%%% Set constants
[samples, ~] = size(X{1}{1});

%%% Check for even windowsize
if mod(windowsize,2)
    windowsize = windowsize+1;
end

for i=1:length(X) % Loops over the number of sensors
    Accel = {};
    window_time = [];
    n=1;
    for j=1+windowsize/2:windowslide:samples-windowsize/2 % Loops through data to create windows
        n_time = X{i}{1}(j,1);                         % timestamp at center of window
        a = X{i}{1}(j-windowsize/2:j+windowsize/2,:);  % acceleration vectors for window
        
        %%% Data to store in struct
        Accel{n} = a;
        window_time = [window_time; n_time];
        n=n+1;
    end
    switch i    % Stores Window data
        case 1
            SensData.Foot.WinAccel = Accel;
            SensData.Foot.WinTime = window_time;
            SensData.Foot.Accel = X{i}{1};
        case 2
            SensData.Shin.WinAccel = Accel;
            SensData.Shin.WinTime = window_time;
            SensData.Shin.Accel = X{i}{1};
        case 3
            SensData.Thigh.WinAccel = Accel;
            SensData.Thigh.WinTime = window_time;
            SensData.Thigh.Accel = X{i}{1};
    end
end

end