%% Function to clip sensor data
%%%This function will take a time vector associated with truth data and
%%%will find corresponding windows in the sensor data. Any windows that do
%%%not have truth data associated with them will be clipped.
%% Inputs:
%%%TruthTimes:   Vector corresponding to the times at the center of each
%%%              window with associated truth
%%%Data:         Struct containing accel and gyro window data for windows
%%%              of individual sensor data
%%%Act:          Struct containing SHOD features for windows of individual
%%%              sensor data

%% Outputs:
%%%DataNew:      Struct containing accel and gyro window data for windows
%%%              with truth data 
%%%ActNew:       Struct containing SHOD features for windows with truth
%%%              data

function [DataNew]=ClipSensorData(TruthTimes, Data) %, ActNew]=ClipSensorData(TruthTimes, Data, Act)
    %Extracts fields from Data struct
    Accel = Data.WinAccel;
    Gyro = Data.WinGyro;
    DataTimes = Data.WinTime;
    %DataSHOD = Act.SHOD;

    %Searches through window data and clips windows with no associated
    %truth
    WinTime = [];
    WinAccel = {};
    WinGyro = {};
    %SHOD = [];
    n = 1;
    for i=1:length(TruthTimes)
        ind = find(abs(DataTimes - TruthTimes(i))<0.001);
        if ind ~= 0
           WinTime = [WinTime; DataTimes(ind)];
           WinAccel{n} = Accel{ind};
           WinGyro{n} = Gyro{ind};
          % SHOD = [SHOD; DataSHOD(ind)];
           n = n + 1;
        end
    end
    DataNew.WinTime = WinTime;
    DataNew.WinAccel = WinAccel';
    DataNew.WinGyro = WinGyro';
    DataNew.Accel = Data.Accel;
    DataNew.Gyro = Data.Gyro;
    %ActNew.SHOD = SHOD;
    
end