%% Data Initialization:  Resample and Gather Data
%%%Will read sensor csv files and create Data.mat files within each data
%%%segment folder. Will then resample all data to uniform time vector and
%%%will create a KickingData.mat file of processed data. Offset between
%%%truth and sensor data has not been applied!
%%%Path to run _Data/ANALYSIS Data/<BabyNumber>/<TestingDate>/

%% Inputs: 

%% Outputs:
%%%Creates Data.mat within each Data folder for each segment of data 
%%%Creates KickingData.mat within each data segment folder


%% Data Initialization
%%% Initialize Constants
close all
ConstantsFlags
files = dir('20*');
for i = 1:length(files)
    
    %%% Creates Data.mat within each Data folder for each segment of data
    cd(files(i).name)
    cd 'Data'
    ReadCSV('Data.mat', Flags.HEADERflag)
    dinfo=dir('Data.mat');
    filename = dinfo.name;
    load(filename,'Data');
    cd ..
    load('TruthLabels.mat')
    
    %%% Processes data within each Data folder and createa KickingData.mat
    %%% file for each segment of data
    KickingData = struct();
    KickingData.XRaw = GatherData(Data, Constants.NumMeas, Flags.HFSflag, Constants.fs);
    
    %%% Find overall starttime and endtime for sensors (i.e. find the
    %%% maximum starttime and minimum endtime for all six sensors. These
    %%% will be used to resample the data to a universal time vector)
    [starttime, endtime] = StartEndTime({KickingData.XRaw{1},KickingData.XRaw{2},...
        KickingData.XRaw{3},KickingData.XRaw{4},KickingData.XRaw{5},...
        KickingData.XRaw{6}});
    
    %%%  Resample data to overall time vector (uniform within sensor data)
    KickingData.XLeftUnfiltMeaned = ResampleData4({KickingData.XRaw{1},KickingData.XRaw{2},...
        KickingData.XRaw{3}},Constants.fs, starttime, endtime);
    KickingData.XRightUnfiltMeaned = ResampleData4({KickingData.XRaw{4},KickingData.XRaw{5},...
        KickingData.XRaw{6}},Constants.fs, starttime, endtime);
    KickingData.XLeftUnfilt = ResampleData4Demean({KickingData.XRaw{1},KickingData.XRaw{2},...
        KickingData.XRaw{3}},Constants.fs, starttime, endtime);
    KickingData.XRightUnfilt = ResampleData4Demean({KickingData.XRaw{4},KickingData.XRaw{5},...
        KickingData.XRaw{6}},Constants.fs, starttime, endtime);
    disp('Done Resampling')
    
%     if Flags.PLOTflag
%         figure()
%         title('Right Thigh Gyro Unfilt Meaned')
%         hold on
%         plot(KickingData.XRightUnfiltMeaned{3}{2}(:,1),KickingData.XRightUnfiltMeaned{3}{2}(:,2))
%         plot(KickingData.XRightUnfiltMeaned{3}{2}(:,1),KickingData.XRightUnfiltMeaned{3}{2}(:,3))
%         plot(KickingData.XRightUnfiltMeaned{3}{2}(:,1),KickingData.XRightUnfiltMeaned{3}{2}(:,4))
%         hold off
%     end
    
    %%%  Filters data if Flags.FILTflag is high
    if Flags.FILTflag
       % KickingData.XLeft=Lowpass(KickingData.XLeftUnfilt, Constants.nLP, Constants.fcLP, Constants.fs);
       % KickingData.XRight=Lowpass(KickingData.XRightUnfilt, Constants.nLP, Constants.fcLP, Constants.fs);
       %KickingData.XLeftMeaned=Lowpass(KickingData.XLeftUnfiltMeaned, Constants.nLP, Constants.fcLP, Constants.fs);
       %KickingData.XRightMeaned=Lowpass(KickingData.XRightUnfiltMeaned, Constants.nLP, Constants.fcLP, Constants.fs);        
       KickingData.XLeftMeaned=MovAvg(KickingData.XLeftUnfiltMeaned, Constants.avgwin);
       KickingData.XRightMeaned=MovAvg(KickingData.XRightUnfiltMeaned, Constants.avgwin);     
       KickingData.XLeft=MovAvg(KickingData.XLeftUnfilt, Constants.avgwin);
       KickingData.XRight=MovAvg(KickingData.XRightUnfilt, Constants.avgwin);
       disp('Done Filtering');
    else
       KickingData.XLeftMeaned= KickingData.XLeftUnfiltMeaned;
       KickingData.XRightMeaned= KickingData.XRightUnfiltMeaned;
       KickingData.XLeft= KickingData.XLeftUnfilt;
       KickingData.XRight= KickingData.XRightUnfilt;
    end

%     KickingData.XLeft = KickingData.XLeftMeaned;
%     KickingData.XRight = KickingData.XRightMeaned;
%     
%     if Flags.PLOTflag
%         figure()
%         title('Right Thigh Gyro Filt')
%         hold on
%         plot(KickingData.XRight{3}{2}(:,1),KickingData.XRight{3}{2}(:,2))
%         plot(KickingData.XRight{3}{2}(:,1),KickingData.XRight{3}{2}(:,3))
%         plot(KickingData.XRight{3}{2}(:,1),KickingData.XRight{3}{2}(:,4))
%         hold off
%     end
%     
    %%%  Gather Different measurement types together
    KickingData.GatheredSensorsL = GatherSensors(KickingData.XLeft, Constants.NumMeas);
    KickingData.GatheredSensorsR = GatherSensors(KickingData.XRight, Constants.NumMeas);
    

    %%% Save Kicking Data as .mat
    save('KickingData.mat','KickingData')
    clear dinfo filename    
    cd ..
    
end