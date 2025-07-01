%% Offset and Crop Data
%%% Note: Nothing in this script will need to be updated.
%%% Also: This code is not pretty, but does function. If you would like to
%%% update it, go for it.

%% Incorporate offset into sensor data
num_dataseg = length(ActData.datasegment);
for i=1:num_dataseg
    %%%Offset WinTime
    tempLT = SensData.datasegment(i).Left.Thigh.WinTime + ActData.datasegment(i).Left.Thigh.Offset;
    tempLS = SensData.datasegment(i).Left.Shin.WinTime + ActData.datasegment(i).Left.Shin.Offset;
    tempLF = SensData.datasegment(i).Left.Foot.WinTime + ActData.datasegment(i).Left.Foot.Offset;
    SensData.datasegment(i).Left.Thigh.WinTime = tempLT;     
    SensData.datasegment(i).Left.Shin.WinTime = tempLS;
    SensData.datasegment(i).Left.Foot.WinTime = tempLF;
    SensData.datasegment(i).LeftMeaned.Thigh.WinTime = tempLT;     
    SensData.datasegment(i).LeftMeaned.Shin.WinTime = tempLS;
    SensData.datasegment(i).LeftMeaned.Foot.WinTime = tempLF;
    
    tempRT = SensData.datasegment(i).Right.Thigh.WinTime + ActData.datasegment(i).Right.Thigh.Offset;    
    tempRS = SensData.datasegment(i).Right.Shin.WinTime + ActData.datasegment(i).Right.Shin.Offset;
    tempRF = SensData.datasegment(i).Right.Foot.WinTime + ActData.datasegment(i).Right.Foot.Offset;
    SensData.datasegment(i).Right.Thigh.WinTime = tempRT;
    SensData.datasegment(i).Right.Shin.WinTime = tempRS;
    SensData.datasegment(i).Right.Foot.WinTime = tempRF;
    SensData.datasegment(i).RightMeaned.Thigh.WinTime = tempRT;
    SensData.datasegment(i).RightMeaned.Shin.WinTime = tempRS;
    SensData.datasegment(i).RightMeaned.Foot.WinTime = tempRF;
    clear tempLT tempLS tempLF tempRT tempRS tempRF
    
    %%%Offset Accel
    tempLT = SensData.datasegment(i).Left.Thigh.Accel(:,1) + ActData.datasegment(i).Left.Thigh.Offset;
    tempLS = SensData.datasegment(i).Left.Shin.Accel(:,1) + ActData.datasegment(i).Left.Shin.Offset;
    tempLF = SensData.datasegment(i).Left.Foot.Accel(:,1) + ActData.datasegment(i).Left.Foot.Offset;
    SensData.datasegment(i).Left.Thigh.Accel(:,1) = tempLT;
    SensData.datasegment(i).Left.Shin.Accel(:,1) = tempLS;
    SensData.datasegment(i).Left.Foot.Accel(:,1) = tempLF;
    SensData.datasegment(i).LeftMeaned.Thigh.Accel(:,1) = tempLT;
    SensData.datasegment(i).LeftMeaned.Shin.Accel(:,1) = tempLS;
    SensData.datasegment(i).LeftMeaned.Foot.Accel(:,1) = tempLF;
    
    tempRT = SensData.datasegment(i).Right.Thigh.Accel(:,1) + ActData.datasegment(i).Right.Thigh.Offset;
    tempRS = SensData.datasegment(i).Right.Shin.Accel(:,1) + ActData.datasegment(i).Right.Shin.Offset;
    tempRF = SensData.datasegment(i).Right.Foot.Accel(:,1) + ActData.datasegment(i).Right.Foot.Offset;
    SensData.datasegment(i).Right.Thigh.Accel(:,1) = tempRT;
    SensData.datasegment(i).Right.Shin.Accel(:,1) = tempRS;
    SensData.datasegment(i).Right.Foot.Accel(:,1) = tempRF;
    SensData.datasegment(i).RightMeaned.Thigh.Accel(:,1) = tempRT;
    SensData.datasegment(i).RightMeaned.Shin.Accel(:,1) = tempRS;
    SensData.datasegment(i).RightMeaned.Foot.Accel(:,1) = tempRF;
    clear tempLT tempLS tempLF tempRT tempRS tempRF
    
    %%%Offset Gyro
    tempLT = SensData.datasegment(i).Left.Thigh.Gyro(:,1) + ActData.datasegment(i).Left.Thigh.Offset;
    tempLS = SensData.datasegment(i).Left.Shin.Gyro(:,1) + ActData.datasegment(i).Left.Shin.Offset;
    tempLF = SensData.datasegment(i).Left.Foot.Gyro(:,1) + ActData.datasegment(i).Left.Foot.Offset;
    SensData.datasegment(i).Left.Thigh.Gyro(:,1) = tempLT;
    SensData.datasegment(i).Left.Shin.Gyro(:,1) = tempLS;
    SensData.datasegment(i).Left.Foot.Gyro(:,1) = tempLF;
    
    tempRT = SensData.datasegment(i).Right.Thigh.Gyro(:,1) + ActData.datasegment(i).Right.Thigh.Offset;
    tempRS = SensData.datasegment(i).Right.Shin.Gyro(:,1) + ActData.datasegment(i).Right.Shin.Offset;
    tempRF = SensData.datasegment(i).Right.Foot.Gyro(:,1) + ActData.datasegment(i).Right.Foot.Offset;
    SensData.datasegment(i).Right.Thigh.Gyro(:,1) = tempRT;
    SensData.datasegment(i).Right.Shin.Gyro(:,1) = tempRS;
    SensData.datasegment(i).Right.Foot.Gyro(:,1) = tempRF;
    clear tempLT tempLS tempLF tempRT tempRS tempRF
    
    %%%Overall WinTime
    mintime = max([SensData.datasegment(i).Left.Thigh.WinTime(1),SensData.datasegment(i).Left.Shin.WinTime(1),SensData.datasegment(i).Left.Foot.WinTime(1),...
        SensData.datasegment(i).Right.Thigh.WinTime(1),SensData.datasegment(i).Right.Shin.WinTime(1),SensData.datasegment(i).Right.Foot.WinTime(1)]);
    maxtime = min([SensData.datasegment(i).Left.Thigh.WinTime(end),SensData.datasegment(i).Left.Shin.WinTime(end),SensData.datasegment(i).Left.Foot.WinTime(end),...
        SensData.datasegment(i).Right.Thigh.WinTime(end),SensData.datasegment(i).Right.Shin.WinTime(end),SensData.datasegment(i).Right.Foot.WinTime(end)]);
    
    WinTime = mintime:0.1:maxtime;
    ActData.datasegment(i).Left.WinTime = WinTime;
    ActData.datasegment(i).Right.WinTime = WinTime;
end

%% Creates truth at each window center and clips sensor data when truth unavailable for a window
%%% Will search through truth data to find truth for each window. If a
%%% window does not have truth data, it will be clipped from the SensData
%%% and Act Data structs.
for i=1:num_dataseg
    %%%Create truth for each window
    [Truth.datasegment(i).Left.WinTime, Truth.datasegment(i).Left.WinTruth] = ...
        CreateTruth(Truth.datasegment(i).Left.TruthResamp, ActData.datasegment(i).Left.WinTime);
    [Truth.datasegment(i).Right.WinTime, Truth.datasegment(i).Right.WinTruth] = ...
        CreateTruth(Truth.datasegment(i).Right.TruthResamp, ActData.datasegment(i).Right.WinTime);
    WinTime = Truth.datasegment(i).Left.WinTime;
    
    %%%Clip sensor data to get rid of windows with no associated truth
    [SensData.datasegment(i).Left.Foot] =...
        ClipSensorData(WinTime, SensData.datasegment(i).Left.Foot);
    [SensData.datasegment(i).Left.Shin]=...
        ClipSensorData(WinTime, SensData.datasegment(i).Left.Shin);
    [SensData.datasegment(i).Left.Thigh]=...
        ClipSensorData(WinTime, SensData.datasegment(i).Left.Thigh);
    
    [SensData.datasegment(i).Right.Foot]=...
        ClipSensorData(WinTime, SensData.datasegment(i).Right.Foot);
    [SensData.datasegment(i).Right.Shin]=...
        ClipSensorData(WinTime, SensData.datasegment(i).Right.Shin);
    [SensData.datasegment(i).Right.Thigh]=...
        ClipSensorData(WinTime, SensData.datasegment(i).Right.Thigh);
    
    %%%Clip sensor data to get rid of windows with no associated truth
    [SensData.datasegment(i).LeftMeaned.Foot] =...
        ClipSensorData2(WinTime, SensData.datasegment(i).LeftMeaned.Foot);
    [SensData.datasegment(i).LeftMeaned.Shin] =...
        ClipSensorData2(WinTime, SensData.datasegment(i).LeftMeaned.Shin);
    [SensData.datasegment(i).LeftMeaned.Thigh] =...
        ClipSensorData2(WinTime, SensData.datasegment(i).LeftMeaned.Thigh);
    
    [SensData.datasegment(i).RightMeaned.Foot] =...
        ClipSensorData2(WinTime, SensData.datasegment(i).RightMeaned.Foot);
    [SensData.datasegment(i).RightMeaned.Shin] =...
        ClipSensorData2(WinTime, SensData.datasegment(i).RightMeaned.Shin);
    [SensData.datasegment(i).RightMeaned.Thigh] =...
        ClipSensorData2(WinTime, SensData.datasegment(i).RightMeaned.Thigh);
    
    [ActData.datasegment(i).Left] =...
        ClipActFeatures(WinTime, ActData.datasegment(i).Left);
    [ActData.datasegment(i).Right] =...
        ClipActFeatures(WinTime, ActData.datasegment(i).Right);
end
clear i WinTime

%% Crop overall sensor data
for i = 1:num_dataseg
    %%%Ensure sorted
    Truth.datasegment(i).Left.TruthResamp = sortrows(Truth.datasegment(i).Left.TruthResamp);
    Truth.datasegment(i).Right.TruthResamp = sortrows(Truth.datasegment(i).Right.TruthResamp);
    %%%Ensure unique time vector
    [~ , rows] = unique(Truth.datasegment(i).Left.TruthResamp(:,1));
    Truth.datasegment(i).Left.TruthResamp = Truth.datasegment(i).Left.TruthResamp(rows,:);
    [~ , rows] = unique(Truth.datasegment(i).Right.TruthResamp(:,1));
    Truth.datasegment(i).Right.TruthResamp = Truth.datasegment(i).Right.TruthResamp(rows,:);    
   
    timestart = max(Truth.datasegment(i).Left.TruthResamp(1,1),SensData.datasegment(i).Left.Foot.Accel(1,1));
    timeend = min(Truth.datasegment(i).Left.TruthResamp(end,1),SensData.datasegment(i).Left.Foot.Accel(end,1));
    
    %%%Crop sensor data
    start_ind = find(abs(SensData.datasegment(i).Left.Foot.Accel(:,1)-timestart)<.005);
    end_ind = find(abs(SensData.datasegment(i).Left.Foot.Accel(:,1)-timeend)<.005);
    
    SensData.datasegment(i).Left.Foot.Accel = SensData.datasegment(i).Left.Foot.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).Right.Foot.Accel = SensData.datasegment(i).Right.Foot.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).Left.Shin.Accel = SensData.datasegment(i).Left.Shin.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).Right.Shin.Accel = SensData.datasegment(i).Right.Shin.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).Left.Thigh.Accel = SensData.datasegment(i).Left.Thigh.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).Right.Thigh.Accel = SensData.datasegment(i).Right.Thigh.Accel(start_ind:end_ind,:);
    
    SensData.datasegment(i).Left.Foot.Gyro = SensData.datasegment(i).Left.Foot.Gyro(start_ind:end_ind,:);
    SensData.datasegment(i).Right.Foot.Gyro = SensData.datasegment(i).Right.Foot.Gyro(start_ind:end_ind,:);
    SensData.datasegment(i).Left.Shin.Gyro = SensData.datasegment(i).Left.Shin.Gyro(start_ind:end_ind,:);
    SensData.datasegment(i).Right.Shin.Gyro = SensData.datasegment(i).Right.Shin.Gyro(start_ind:end_ind,:);
    SensData.datasegment(i).Left.Thigh.Gyro = SensData.datasegment(i).Left.Thigh.Gyro(start_ind:end_ind,:);
    SensData.datasegment(i).Right.Thigh.Gyro = SensData.datasegment(i).Right.Thigh.Gyro(start_ind:end_ind,:);
    
    SensData.datasegment(i).LeftMeaned.Foot.Accel = SensData.datasegment(i).LeftMeaned.Foot.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).RightMeaned.Foot.Accel = SensData.datasegment(i).RightMeaned.Foot.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).LeftMeaned.Shin.Accel = SensData.datasegment(i).LeftMeaned.Shin.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).RightMeaned.Shin.Accel = SensData.datasegment(i).RightMeaned.Shin.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).LeftMeaned.Thigh.Accel = SensData.datasegment(i).LeftMeaned.Thigh.Accel(start_ind:end_ind,:);
    SensData.datasegment(i).RightMeaned.Thigh.Accel = SensData.datasegment(i).RightMeaned.Thigh.Accel(start_ind:end_ind,:);
    
    %%%Crop Truth
    TempTime = SensData.datasegment(i).Left.Foot.Accel(:,1);
    TempTruthL = round(interp1(Truth.datasegment(i).Left.TruthResamp(:,1),Truth.datasegment(i).Left.TruthResamp(:,2),TempTime,'pchip'));
    TempTruthR = round(interp1(Truth.datasegment(i).Right.TruthResamp(:,1),Truth.datasegment(i).Right.TruthResamp(:,2),TempTime,'pchip'));
    
    Truth.datasegment(i).Left.TruthResamp = [];
    Truth.datasegment(i).Right.TruthResamp = [];
    
    Truth.datasegment(i).Left.TruthResamp(:,1) = TempTime;
    Truth.datasegment(i).Right.TruthResamp(:,1) = TempTime;
    Truth.datasegment(i).Left.TruthResamp(:,2) = TempTruthL;
    Truth.datasegment(i).Right.TruthResamp(:,2) = TempTruthR;
end
clear timestart timeend start_ind end_ind

%% Resample SHOD to time vector
for i=1:num_dataseg
    ActData.datasegment(i).Left.Time = SensData.datasegment(i).Left.Foot.Accel(:,1);
    ActData.datasegment(i).Left.SHOD3D_Resamp = interp1(ActData.datasegment(i).Left.WinTime,ActData.datasegment(i).Left.SHOD3D,ActData.datasegment(i).Left.Time,'spline');
    ActData.datasegment(i).Left.Thigh.SHOD_Resamp = interp1(ActData.datasegment(i).Left.WinTime,ActData.datasegment(i).Left.Thigh.SHOD,ActData.datasegment(i).Left.Time,'spline');
    ActData.datasegment(i).Left.Shin.SHOD_Resamp = interp1(ActData.datasegment(i).Left.WinTime,ActData.datasegment(i).Left.Shin.SHOD,ActData.datasegment(i).Left.Time,'spline');
    ActData.datasegment(i).Left.Foot.SHOD_Resamp = interp1(ActData.datasegment(i).Left.WinTime,ActData.datasegment(i).Left.Foot.SHOD,ActData.datasegment(i).Left.Time,'spline');
    
    ActData.datasegment(i).Right.Time = SensData.datasegment(i).Right.Foot.Accel(:,1);
    ActData.datasegment(i).Right.SHOD3D_Resamp = interp1(ActData.datasegment(i).Right.WinTime,ActData.datasegment(i).Right.SHOD3D,ActData.datasegment(i).Right.Time,'spline');
    ActData.datasegment(i).Right.Thigh.SHOD_Resamp = interp1(ActData.datasegment(i).Right.WinTime,ActData.datasegment(i).Right.Thigh.SHOD,ActData.datasegment(i).Right.Time,'spline');
    ActData.datasegment(i).Right.Shin.SHOD_Resamp = interp1(ActData.datasegment(i).Right.WinTime,ActData.datasegment(i).Right.Shin.SHOD,ActData.datasegment(i).Right.Time,'spline');
    ActData.datasegment(i).Right.Foot.SHOD_Resamp = interp1(ActData.datasegment(i).Right.WinTime,ActData.datasegment(i).Right.Foot.SHOD,ActData.datasegment(i).Right.Time,'spline');
end

save('GatheredData_Crop.mat', 'ActData', 'SensData', 'Truth')
clearvars -except Constants Flags