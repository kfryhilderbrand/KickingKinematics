%% Constants and Flags to adjust
Flags = struct();
Constants = struct();

%%%%%% Flags to adjust
Flags.HEADERflag = 0; %csv files have headers
Flags.PLOTflag = 0;  %Want to generate plots for testing
Flags.HFSflag = 1;   %Flag determining if High Frequency Streaming was used (0 for no). If 1, 
                      ...then timestamps within each packet of data will be adjusted.

%%%%%% Constants to adjust
%General Constants
Constants.fs = 100; %sampling frequency in Hz 
Constants.NumSens = 6; %How many total sensors
Constants.NumMeas = 2; %How many measurement files per sensor (Accel and Gyro in this case)

%Constants for moving avg
Constants.avgwin = 15;

%For Activity Identifier
Constants.win = 250; %window in ms
Constants.step = 100; %step for sliding window
Constants.windowsize = floor((Constants.win/1000)*Constants.fs); %approximate number of samples for window
                                                                 ...must be integer
Constants.windowslide = floor((Constants.step/1000)*Constants.fs); %approximate number of smples for the window slide
Constants.g = 1; %in g's
Constants.VarAccelNoise = 3.24 * 10^-6; % of sensors
Constants.VarGyroNoise = .0049; % of sensors

%For Median Smoothing Filter
Constants.width = 15;

%For Data Segmenting
Constants.SegLength = 60; %s
Constants.SegStep = 60; %s

%Phase Lookup Table
%%% Left leg lookup table with numerical phase code equivalent (1, -1, 0)
Constants.Lookup.Left.HF = {'flexion','extension','NM'};
Constants.Lookup.Left.HA = {'adduction','abduction','NM'};
Constants.Lookup.Left.HR = {'internal_rot','external_rot','NM'};
Constants.Lookup.Left.KF = {'extension','flexion','NM'};
Constants.Lookup.Left.AF = {'flexion','extension','NM'};
Constants.Lookup.Left.AI = {'eversion','inversion','NM'};

%%% Right leg lookup table with numerical phase code equivalent (1, -1, 0)
Constants.Lookup.Right.HF = {'flexion','extension','NM'};
Constants.Lookup.Right.HA = {'abduction','adduction','NM'};
Constants.Lookup.Right.HR = {'external_rot','internal_rot','NM'};
Constants.Lookup.Right.KF = {'extension','flexion','NM'};
Constants.Lookup.Right.AF = {'flexion','extension','NM'};
Constants.Lookup.Right.AI = {'inversion','eversion','NM'};