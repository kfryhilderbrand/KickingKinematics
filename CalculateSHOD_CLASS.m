%% Function to calculate SHOD features for all data
%%%This function will use a stance hypothesis optimal detector (SHOD) to
%%%analyze both acceleration and angular velocity. Will create a struct to
%%%store SHOD vectors for each sensor and a 3D SHOD feature vector for one
%%%leg of sensors. Will also store window data at each timestamp that the
%%%SHOD value is calculated.
%% Inputs:
%%%X:           Data to classify

%% Outputs:
%%%SensData:    Struct containing window data and timestamps for center of
%%%             windows
%%%ActData:     Struct containing SHOD feature vectors

function [SensData, ActData]=CalculateSHOD_CLASS(X)

%%%Set constants
ConstantsFlags
windowsize = Constants.windowsize;
windowslide = Constants.windowslide;
VarAccelNoise = Constants.VarAccelNoise;
VarGyroNoise = Constants.VarGyroNoise;
g = Constants.g;
[samples, ~] = size(X{1}{1});

%%%Check for even windowsize
if mod(windowsize,2)
    windowsize = windowsize+1;
end

for i=1:length(X) %%%Loops over the number of sensors
    SHOD = [];
    Accel = {};
    Gyro = {};
    window_time = [];
    n=1;
    for j=1+windowsize/2:windowslide:samples-windowsize/2 %%%Loops through data to create windows
        n_time = X{i}{1}(j,1);                         %timestamp at center of window
        a = X{i}{1}(j-windowsize/2:j+windowsize/2,:);  %acceleration vectors for window
        w = X{i}{2}(j-windowsize/2:j+windowsize/2,:);  %angular velocity vectors for window
        an_bar = [mean(a(:,2)),mean(a(:,3)),mean(a(:,4))];
        
        sum = 0;
        for k=1:windowsize %%%Sums values within window to create SHOD value
            sum = sum + (1/(VarAccelNoise))*norm(a(k,2:4)-g*(an_bar/norm(an_bar)))^2+...
                (1/(VarGyroNoise))*norm(w(k,2:4))^2;
        end
        
        if isnan(sum)
            sum = 0;
        end
        
        %%%Data to store in struct
       % if (n>1)
       %     if (n_time~=window_time(n-1))
%                 SHOD = [SHOD;(1/windowsize)*sum];
%                 Accel{n} = a;
%                 Gyro{n} = w;
%                 window_time = [window_time; n_time];
%                 n=n+1;
%             end
       % else
            SHOD = [SHOD;(1/windowsize)*sum];
            Accel{n} = a;
            Gyro{n} = w;
            window_time = [window_time; n_time];
            n=n+1;
       % end
    end

    switch i    %%%Stores Window data and 1D SHOD feature vectors
        case 1
            ActData.Foot.SHOD = SHOD;
            SensData.Foot.WinAccel = Accel;
            SensData.Foot.WinGyro = Gyro;
            SensData.Foot.WinTime = window_time;
            SensData.Foot.Accel = X{i}{1};
            SensData.Foot.Gyro = X{i}{2};
        case 2
            ActData.Shin.SHOD = SHOD;
            SensData.Shin.WinAccel = Accel;
            SensData.Shin.WinGyro = Gyro;
            SensData.Shin.WinTime = window_time;
            SensData.Shin.Accel = X{i}{1};
            SensData.Shin.Gyro = X{i}{2};
        case 3
            ActData.Thigh.SHOD = SHOD;
            SensData.Thigh.WinAccel = Accel;
            SensData.Thigh.WinGyro = Gyro;
            SensData.Thigh.WinTime = window_time;
            SensData.Thigh.Accel = X{i}{1};
            SensData.Thigh.Gyro = X{i}{2};
    end
end

% SHOD = [];
% SHOD_RMS = [];
% for i = 1:length(SensData.Foot.WinTime)
%    temp = [ActData.Foot.SHOD(i),ActData.Shin.SHOD(i),ActData.Thigh.SHOD(i)];
%    SHOD = [SHOD; norm(temp)];
%    SHOD_RMS = [SHOD_RMS; rms(temp)];
% end
% 
% ActData.SHOD3D = SHOD;
% ActData.SHOD3D_RMS = SHOD_RMS;

ActData.WinTime = window_time;

end