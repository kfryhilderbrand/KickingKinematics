%% Function to rename sensor files to start with their placement
%%% Path to files _Data/ANALYSIS Data/<BabyNumber>/<TestingDate>/<Segment>
%%%If sensor data is missing, will create dummy file to replace it.
%%%With new functionality in the app, this function may become obsolete.
%% Inputs: 
%%%     sensorplacement: struct with six values indicating sensor placement 
%%%                     for each segment of test data.
%%%     optional: placeholdertime (if you know files are missing, this will 
%%%               put those vectors as placeholder time), 2xn
%% Outputs:

function RenameSensorFiles(sensorplacement, placeholdertime)

if ~exist('placeholdertime','var')
     %%%If placeholdertime not specified, then placeholdertime set to the
     %%%below.
     
     placeholdertime = [zeros(3,5);...
                       [1E10;1E10;1E10], [1E10;1E10;1E10], zeros(3,3)];
                         
end


%%% Variables
Name{1} = sensorplacement.LF;
Name{2} = sensorplacement.LS;
Name{3} = sensorplacement.LT;
Name{4} = sensorplacement.RF;
Name{5} = sensorplacement.RS;
Name{6} = sensorplacement.RT;

NewName{1} = 'LF_ACCEL.csv';
NewName{2} = 'LF_GYRO.csv';
NewName{3} = 'LS_ACCEL.csv';
NewName{4} = 'LS_GYRO.csv';
NewName{5} = 'LT_ACCEL.csv';
NewName{6} = 'LT_GYRO.csv';
NewName{7} = 'RF_ACCEL.csv';
NewName{8} = 'RF_GYRO.csv';
NewName{9} = 'RS_ACCEL.csv';
NewName{10} = 'RS_GYRO.csv';
NewName{11} = 'RT_ACCEL.csv';
NewName{12} = 'RT_GYRO.csv';


dinfo=dir('20*');
names_cell = {dinfo.name};

checkfiles = cell(1,length(names_cell));
for m=1:length(names_cell)
    cd(names_cell{m})
    
    n = length(placeholdertime(:,1));
    placeholder = [placeholdertime(:,1), placeholdertime(:,2), zeros(n,1), zeros(n,1), zeros(n,1)];
    
    cd('Data')
    csvinfo=dir('*.csv');
    csvnames = {csvinfo.name};
    
    if length(csvnames)~=12   %%%if not all sensor data present
        missing = zeros(1,12);
        for j=1:length(csvnames)
            temp = zeros(1,12);
            sens = 1;
            for k = 1:12  %%%will compare all csv names to provided sensor names
                if mod(k,2) %%%if k odd
                    temp(k) = strcmp(csvnames{j}(1:2),Name{sens})&&strcmp(csvnames{j}(end-9+1:end),'ACCEL.csv');
                else
                    temp(k) = strcmp(csvnames{j}(1:2),Name{sens})&&strcmp(csvnames{j}(end-8+1:end),'GYRO.csv');
                    sens = sens + 1;
                end
            end
            missing(find(temp))=1; %#ok<FNDSB>
        end
        
        ind = find(missing==0);
        for j=1:length(ind)
            CreateFLAG = 0;  %%Checks for sensor file already renamed
            for k=1:length(csvnames)
                if strcmp(csvnames{k}(1:2),NewName{ind(j)}(1:2))&&strcmp(csvnames{k}(end-9+1:end),NewName{ind(j)}(end-9+1:end))
                    CreateFLAG = 1;
                end
            end
            if ~CreateFLAG
                csvwrite(NewName{ind(j)},placeholder);
            end
        end
        checkfiles{m} = 'placeholder files created';
    end
    
    %%% reset csvinfo and csvnames after new files created
    csvinfo=dir('*.csv');
    csvnames = {csvinfo.name};
    namecomp = zeros(1,6);
    comp = zeros(1,6);
    
    for n=1:length(csvnames)
        
        %%% See if files already renamed
        namecomp(1) = strcmp(csvnames{n}(1:5),sprintf('LF_%s',sensorplacement.LF));
        namecomp(2) = strcmp(csvnames{n}(1:5),sprintf('LS_%s',sensorplacement.LS));
        namecomp(3) = strcmp(csvnames{n}(1:5),sprintf('LT_%s',sensorplacement.LT));
        namecomp(4) = strcmp(csvnames{n}(1:5),sprintf('RF_%s',sensorplacement.RF));
        namecomp(5) = strcmp(csvnames{n}(1:5),sprintf('RS_%s',sensorplacement.RS));
        namecomp(6) = strcmp(csvnames{n}(1:5),sprintf('RT_%s',sensorplacement.RT));
        
        ind = find(namecomp, 1);
        if isempty(ind)      %%If file hasn't already been renamed
            
            comp(1) = strcmp(csvnames{n}(1:2),sensorplacement.LF);
            comp(2) = strcmp(csvnames{n}(1:2),sensorplacement.LS);
            comp(3) = strcmp(csvnames{n}(1:2),sensorplacement.LT);
            comp(4) = strcmp(csvnames{n}(1:2),sensorplacement.RF);
            comp(5) = strcmp(csvnames{n}(1:2),sensorplacement.RS);
            comp(6) = strcmp(csvnames{n}(1:2),sensorplacement.RT);
            
            ind = find(comp);
            if isempty(ind)   %%If sensor file can't be found
                if ~strcmp(checkfiles{m},'placeholder files created') %%if temp file was not created
                    checkfiles{m} = 'Sensor file not found';
                end
            else
                switch ind
                    case 1
                        str = sprintf('LF_%s',csvinfo(n).name);
                    case 2
                        str = sprintf('LS_%s',csvinfo(n).name);
                    case 3
                        str = sprintf('LT_%s',csvinfo(n).name);
                    case 4
                        str = sprintf('RF_%s',csvinfo(n).name);
                    case 5
                        str = sprintf('RS_%s',csvinfo(n).name);
                    case 6
                        str = sprintf('RT_%s',csvinfo(n).name);
                end
                movefile(csvinfo(n).name,str);
            end
        else
            checkfiles{m} = 'Already renamed';
        end
    end
    cd ..
    cd ..
end
end