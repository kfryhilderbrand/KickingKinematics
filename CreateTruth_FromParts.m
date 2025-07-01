%% Function to take truth part files from codification and create truth vectors
%%% Resamples codification part files before concatinating parts for overall truth
%%% Creates TruthLabels.mat for each video. TruthLabels is a struct that
%%% contains resampled TruthLeft, TruthRight, and TruthBilat vectors.
%% Inputs: 
%%%     files: struct indicating how many part files per data segment
%%%     fs: sampling frequency to resample truth data

%% Outputs:

function [] = CreateTruth_FromParts(files, fs)

TruthLeft=[];
TruthRight=[];

%%%Load csv files in part folders and resample individual truth parts
for i = 1:length(files)
    if i<10
        path = sprintf('pt0%d',i);
        cd(path)
        mat_Name = sprintf('Truthpt0%d.mat',i);
        mat_NameResamp = sprintf('TruthLabelspt0%d.mat',i);
    else
        path = sprintf('pt%d',i);
        cd(path)
        mat_Name = sprintf('Truthpt%d.mat',i);
        mat_NameResamp = sprintf('TruthLabelspt%d.mat',i);
    end
    ReadCSV_Truth()
    %ReadXLSX_Truth(mat_Name) %%%For old codification files
    
    dinfo=dir(mat_Name);
    filename = dinfo.name;
    load(filename,'Truth');
    
    %%% Create and Resample Truth vector
    [TruthptLabels.Truth,TruthptLabels.TruthResamp] = Resample_Truth(fs, Truth);
    disp('Done Truth Resampling')
    
    %%% Divide Truth into Left and Right parts
    %   (To view individual truth parts, change function call from 0 to 1)
    [TruthptLabels.TruthLeft,TruthptLabels.TruthRight] = Truth_LeftRight(TruthptLabels.TruthResamp, 0);
    disp('Truth Created')
    
    %%Create truth vector for each leg
    TruthLeft=[TruthLeft; TruthptLabels.TruthLeft];
    TruthRight=[TruthRight; TruthptLabels.TruthRight];
    
    save(mat_NameResamp,'TruthptLabels')
    cd ..
    clear dinfo filename Truth TruthLabels mat_Name mat_NameResamp
end
%%%%%Create left and right truth
TruthLabels.TruthLeft = TruthLeft;
TruthLabels.TruthRight = TruthRight;

%%%%%Create bilateral truth
TruthLabels.TruthBilat(:,1) = TruthLeft(:,1);
TruthLabels.TruthBilat(:,2) = TruthLeft(:,2) + 2.*TruthRight(:,2);

%%%%Filter out non-unique time values
[~,indx]=unique(TruthLabels.TruthLeft(:,1));
%             %% USE IF THERE IS A REPEATED VALUE IN TRUTH FILES
%             A=Truth{1}{1}(:,1);
%             [n, bin] = histc(A, unique(A));
%             multiple = find(n > 1);
%             index    = find(ismember(bin, multiple));
TruthLabels.TruthLeft = TruthLabels.TruthLeft(indx,:);
TruthLabels.TruthRight = TruthLabels.TruthRight(indx,:);
TruthLabels.TruthBilat = TruthLabels.TruthBilat(indx,:);

cd ..
save('TruthLabels.mat','TruthLabels')

end