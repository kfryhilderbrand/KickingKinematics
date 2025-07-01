%% Main Script for Infant Kicking Data Analysis
%%% This script will handle everything from preprocessing, activity 
%%% detection, joint axis and joint rate detection, and the determination 
%%% of kicking metrics for a session of infant data. I have tried to include 
%%% as much explanation and detail as possible while keeping everything as 
%%% straightforward as possible.

%%% There are three things you will be required to update in this script.
%%% Everything else is automated and should not need to change.
%   1: in Preprocessing Steps, will need to set the sensor placement to
%   update the names of the sensor files
%   2: in Determine Sensor Offset, will need to determine the offset
%   between the truth data and sensor data. The "offset" variable will need
%   to be tuned, then recorded in "__Data\Video Processing and Codification.xlsx" 
%   for reference.
%   3: in Joint Axis Detection, will need to verify the columns of the PCA
%   matrices are being assigned to the correct DOFs as joint axes (read the
%   instructions in that section).

%% Load Constants
ConstantsFlags

%% Preprocessing Steps

%%MAKE SURE TO SET APPROPRIATE SENSOR PLACEMENT
sensorplacement.LF = 'C3';
sensorplacement.LS = 'EE';
sensorplacement.LT = 'DD';
sensorplacement.RF = 'C2';
sensorplacement.RS = 'D5';
sensorplacement.RT = 'F8';


%%% By the end of this step, you will have a file called
%%% "GatheredData.mat". This file contains multiple structs:
%   Truth: Truth Data for activity detection
%   ActData: Data associated with activity detection
%   SensData: Data associated with sensor data (processed data and windows
%   of data)
Preprocessing

%% Determine Sensor Offset
%%% Will plot truth and sensor data against each other to determine the
%%% offset. 
%   THIS WILL NEED TO BE DETERMINED MANUALLY.
%   If you have properly collected a calibration segment, this should be
%   trivial (i.e. use that calibration segment to determine the offset). If
%   you did not collect a calibration segment, then the offset will need to
%   be determined using gathered data segments (CODE CURRENTLY SET TO DO
%   THIS!)
load('GatheredData.mat')
close all

%%% YOU WILL NEED TO UPDATE THIS 
offset = 0;

%%% YOU PROBABLY WON'T NEED THESE, BUT JUST IN CASE
%   (These are only needed if for some reason your sensors are offset from
%   each other).
Thighoffset = 0;
Shinoffset = 0;
Footoffset = 0;

num_dataseg = length(ActData.datasegment);
for i=1:num_dataseg
 
    TruthSensOffset(Thighoffset, Shinoffset, Footoffset, ActData.datasegment(i).Left, Truth.datasegment(i).Left.TruthResamp)
    
    ActData.datasegment(i).Left.Thigh.Offset = Thighoffset;
    ActData.datasegment(i).Left.Shin.Offset = Shinoffset;
    ActData.datasegment(i).Left.Foot.Offset = Footoffset;
    
    TruthSensOffset(Thighoffset, Shinoffset, Footoffset, ActData.datasegment(i).Right, Truth.datasegment(i).Right.TruthResamp)
    
    ActData.datasegment(i).Right.Thigh.Offset = Thighoffset;
    ActData.datasegment(i).Right.Shin.Offset = Shinoffset;
    ActData.datasegment(i).Right.Foot.Offset = Footoffset;
end

%% Run once offsets have been determined
save('GatheredData.mat', 'ActData', 'SensData', 'Truth')
clearvars -except Constants Flags
close all

%% Offset and Crop Data
load('GatheredData.mat')

%%% By the end of this step, you will have a file called
%%% "GatheredData_Crop.mat". This file contains cropped and offset versions
%%% of the data stored in "GatherData.mat". This file is mainly used for
%%% error checking.
OffsetCropData

%% Activity Detection
load('GatheredData_Crop.mat')

%%% By the end of this step, you will have multiple files saved.
%%% "Truth.mat" and "SensorData.mat" contain data that should match what is
%%% stored in "GatheredData_Crop.mat".
%   "Truth.mat" contains struct 'Truth' with overall truth.
%   "SensorData.mat" contains struct 'SensData' with processed sensor data.
%   "ActivityData.mat" contains struct 'ActData' with SHOD values and
%   predictions of activity for the overall leg and each limb segment.
%   "TrainingData.mat" contains struct 'TrainData' with training data for
%   each datasegment (using leave-one-out method).
ThresholdActDet

%% Joint Axis Detection

%%% By the end of this step, you will have a new struct called 'Axes' which
%%% contains the Hip, Knee, and Ankle joint axes (as determined from PCA).
%%% Each joint will have an associated 3x3 matrix whose columns are the
%%% principle components as determined by PCA. Additionally, the amount of
%%% variance of the data explained by each component is given. 

%%% Additionally, you will have joint axes assigned to each DOF using these
%%% 3x3 matrices. For each joint with K DOFs, the first K columns associated 
%%% with the joint are chosen. For the Hip joint, all three columns are 
%%% are chosen as the joint axes. For the Knee joint, the first column is 
%%% chosen as the joint axis. For the Ankle joint, the first two columns 
%%% are chosen as the joint axes.
JointAxisDet

%%% Before determining the joint rates, the individual joint axes will need 
%%% to be verified. If sensors were placed correctly, the joint axes found 
%%% using PCA should be close to the following (but not necessarily so):
%   HF: [0; 1; 0]
%   HA: [0; 0; 1]
%   HR: [1; 0; 0]
%   KF: [0; 1; 0]
%   AF: [0; 1; 0]
%   AI: [1; 0; 0]
%%% If in doubt, the flexion axes tend to explain the most variance in the
%%% data (i.e. they will be the first column). Additionally, HR tends to
%%% explain the least variance of the Hip (i.e. will tend to be the last
%%% column);

%%% CONFIRM THESE ARE ACCURATE BEFORE DETERMINING JOINT RATES AND CHANGE AS
%%% NEEDED. CHECK BOTH LEGS.
%   Use the following to update an individual DOF: 
%          Axes.Left.HF = Axes.Left.Hip(:,1);
%   Change the leg, DOF, joint, and column number as needed.

%% Joint Rate Determination

%%% By the end of this step, you will have a new file called
%%% "JointRates.mat" which contains a struct called 'rates'. Within rates
%%% are the decoupled joint rates for each DOF.
JointRateDet

%% Segment data

%%% By the end of this step, you will have a new file called
%%% "DataSegments.mat" which contains three structs:
%   SegData: segmented sensor data
%   SegTruth: segmented truth data
%   SegRates: segmented rate data
DataSegments

%% Kicking Kinematic Metrics

%%% By the end of this step, you will have a new file called "PhaseData.mat" 
%%% which contains two structs:
%   phases: coded phases for each DOF
%   phase_names: named phases for each DOF

%%% You will also have a new file called "Metrics.mat" which contains two 
%%% structs: Movement and Metrics. See definitions within
%%% "CalculateKickingMetrics" for more information.

%%% Finally, this step will generate 7 tables of metrics: 
%   Frequency_Metrics: contains 5 frequency metrics
%   Duration_Metrics: contains 10 duration metrics
%   Acceleration_Metrics: contains 36 acceleration metrics
%   Kick_Amp_Metrics: contains 24 kick amplitude metrics
%   Joint_Exc_Metrics:contains 48 joint excursion metrics
%   Joint_Rate_Metrics: contains 24 joint rate metrics
%   Inter_Joint_Metrics: contains 10 inter-joint coordination metrics
CalculateKickingMetrics

%% Wrapping Up
%%% Data from the 7 tables of metrics should be copied into "__Data\Kicking
%%% Metrics.xlsx" for term infants and in "__Data\Kicking Metrics Pre.xlsx"

%%% From my thesis work, a correlation analysis determined multiple metrics
%%% were significantly correlated with age. From this analysis and using an
%%% automated regression procedure, 3 metrics were included in the final
%%% model. With the inclusion of more data, these analyses may need to be
%%% rerun. The correlation analysis and automated regression procedure were
%%% performed in "R" using median values of the metrics stored in "__Data\Kicking
%%% Metrics.xlsx" for each data session. Only term infant data was used in 
%%% the correlation analysis model creation.