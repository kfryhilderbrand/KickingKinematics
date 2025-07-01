%% Function to clip sensor data (Demeaned Accel)
%%%
%% Inputs:
%%%TruthTimes:   Vector corresponding to the times at the center of each
%%%              window with associated truth
%%%Data:         Struct containing accel and gyro window data for windows
%%%              of individual sensor data

%% Outputs:
%%%DataNew:      Struct containing accel and gyro window data for windows
%%%              with truth data 
%%%ActNew:       Struct containing SHOD features for windows with truth
%%%              data

function [DataNew]=ClipSensorData2(TruthTimes, Data)
    %Extracts fields from Data struct
    Accel = Data.WinAccel;
    DataTimes = Data.WinTime;
   
    %Searches through window data and clips windows with no associated
    %truth
    WinTime = [];
    WinAccel = {};

    n = 1;
    for i=1:length(TruthTimes)
        ind = find(abs(DataTimes - TruthTimes(i))<0.001);
        if ind ~= 0
           WinTime = [WinTime; DataTimes(ind)];
           WinAccel{n} = Accel{ind};
           n = n + 1;
        end
    end
    DataNew.WinTime = WinTime;
    DataNew.WinAccel = WinAccel';
    DataNew.Accel = Data.Accel;
   
end