%% Create Data Segments
%%% Note: Nothing in this script will need to be updated.

%% Segment Sensor Data and Truth
[SegData, SegTruth]=Segment_Data(SensData, Truth, ActData, Constants.SegLength, Constants.SegStep, Constants.fs);

%% Segment Joint Rate Data
SegRates = SegmentRateData(rates, Constants.SegLength, Constants.SegStep, Constants.fs);

%% Save structs
save('DataSegments.mat','SegTruth','SegData','SegRates')

clearvars -except Constants Flags SensData ActData Truth rates SegTruth SegData SegRates