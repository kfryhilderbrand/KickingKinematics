%% Preprocessing Steps
%%% Note: Nothing in this script will need to be updated.

%% Sensor File Preprocessing
%%% Renames sensor files to include placement in the name of the file
%   Navigate to data session 
%   Path to files _Data/ANALYSIS Data/<BabyNumber>/<TestingDate>/
vid_files = dir('20*');
for i=1:length(vid_files)
    cd(vid_files(i).name)
    cd('Data')
    Rename_SensorFiles(sensorplacement)
    cd ..
    cd ..
end

clear sensorplacement i

%% Truth File Preprocessing
%%% Resamples and concatenates video data into overall truth for each part:
%%% Also, gathers truth data and create a Truth Struct
%   Navigate to ANALYSIS Data/<BabyNumber>/<TestingDate>/ and run this
%   section.
vid_files = dir('20*');
for i=1:length(vid_files)
    cd(vid_files(i).name)
    cd('Truth')
    files = dir('pt*');
    CreateTruth_FromParts(vid_files, Constants.fs);
    
    load('TruthLabels.mat')
    Truth.datasegment(i).Left.TruthResamp = TruthLabels.TruthLeft;
    Truth.datasegment(i).Right.TruthResamp = TruthLabels.TruthRight;
    clear TruthLabels
    cd ..
end

clear files i vid_files

%% Resample and gather sensor data
%%% Will read sensor csv files and then resample sensor data to uniform time
%%% vector. Also demeans and filters data using a moving average filter.
%%% Finally, saves sensor data (raw and processed) into KickingData.mat file.
%   Path to run _Data/ANALYSIS Data/<BabyNumber>/<TestingDate>/
vid_files = dir('20*');
for i=1:length(vid_files)
    cd(vid_files(i).name)
    cd('Data')
    
    %%% Creates Data.mat within each Data folder for each segment of data
    ReadCSV('Data.mat', Flags.HEADERflag)
    dinfo=dir('Data.mat');
    filename = dinfo.name;
    load(filename,'Data');
    cd ..
    load('TruthLabels.mat')
    
    %%% Gathers data from each Data folder and creates a KickingData.mat
    %%% file for each segment of data
    KickingData = struct();
    KickingData.XRaw = GatherData(Data, Constants.NumMeas, Flags.HFSflag, Constants.fs);
    
    %%% Find overall starttime and endtime for sensors 
    %   These will be used to create overall time vector to resample data to.
    [starttime, endtime] = StartEndTime({KickingData.XRaw{1},KickingData.XRaw{2},...
       KickingData.XRaw{3},KickingData.XRaw{4},KickingData.XRaw{5},...
       KickingData.XRaw{6}});
   
    %%% Resample data to overall time vector (uniform within sensor data)
    %   Overall time vector specified by starttime, endtime and sampling
    %   frequency
    [KickingData.XLeftUnfiltMeaned, KickingData.XLeftUnfilt] = Resample_Data({KickingData.XRaw{1},KickingData.XRaw{2},...
        KickingData.XRaw{3}}, Constants.fs, starttime, endtime);
    [KickingData.XRightUnfiltMeaned, KickingData.XRightUnfilt] = Resample_Data({KickingData.XRaw{4},KickingData.XRaw{5},...
        KickingData.XRaw{6}},Constants.fs, starttime, endtime);
    disp('Done Resampling')
    
    %%%  Filters data using moving average filter
    %    Accounts for movement artifacts and sensor noise
    KickingData.XLeftMeaned=MovAvg(KickingData.XLeftUnfiltMeaned, Constants.avgwin);
    KickingData.XRightMeaned=MovAvg(KickingData.XRightUnfiltMeaned, Constants.avgwin);
    KickingData.XLeft=MovAvg(KickingData.XLeftUnfilt, Constants.avgwin);
    KickingData.XRight=MovAvg(KickingData.XRightUnfilt, Constants.avgwin);
    disp('Done Filtering');
     
    %%%  Gather Different measurement types together
    KickingData.GatheredSensorsL = GatherSensors(KickingData.XLeft, Constants.NumMeas);
    KickingData.GatheredSensorsR = GatherSensors(KickingData.XRight, Constants.NumMeas);
    disp('Done Gathering')
    
    %%% Save Kicking Data as .mat
    save('KickingData.mat','KickingData')
    
    clear dinfo filename KickingData
    cd ..
end

%% Create SensData and ActData struct
%%% Will parse through preprocessed kicking data stored in KickingData struct 
%%% and create windows of data to store in SensData struct. Will also create 
%%% SHOD features using these windows of data to store in ActData struct 
%%% (SHOD features used for activity detection). Finally, will parse through 
%%% non-demeaned accelerometer data to create windows of data to store in 
%%% SensData (this is important for the joint axis and joint rate determinations).
vid_files = dir('20*');
for i = 1:length(vid_files) %%%Loops over data segments (aka data folders)
    cd(vid_files(i).name)
    load('KickingData.mat','KickingData')
    
    %%% Creates windows of data and calculate SHOD features for each window
    %%% of data (SHOD uses both accelerometer and gyroscope data).
    [SensData.datasegment(i).Left, ActData.datasegment(i).Left]=CalculateSHOD_CLASS(KickingData.XLeft);
    [SensData.datasegment(i).Right, ActData.datasegment(i).Right]=CalculateSHOD_CLASS(KickingData.XRight);
    
    %%% Creates windows of non-demeaned accelerometer data (which will be
    %%% used in joint axis and joint rate determinations).
    [SensData.datasegment(i).LeftMeaned]=ParseAccelData(KickingData.XLeftMeaned, Constants.windowsize, Constants.windowslide);
    [SensData.datasegment(i).RightMeaned]=ParseAccelData(KickingData.XRightMeaned, Constants.windowsize, Constants.windowslide);
    
    %%% Creates 3D SHOD Data
    [ActData.datasegment(i).Left.SHOD3D] = Calculate3DSHOD_CLASS(ActData.datasegment(i).Left, SensData.datasegment(i).Left, ActData.datasegment(i).Left.WinTime);
    [ActData.datasegment(i).Right.SHOD3D] = Calculate3DSHOD_CLASS(ActData.datasegment(i).Right, SensData.datasegment(i).Right, ActData.datasegment(i).Right.WinTime);
    
    cd ..
    clear KickingData
end

save('GatheredData.mat', 'ActData', 'SensData', 'Truth')
clearvars -except Constants Flags