%% Function to rename sensor files to start with their placement
%%% Path to files _Data/ANALYSIS Data/<BabyNumber>/<TestingDate>/<Segment>
%%%If sensor data is missing, will create dummy file to replace it.
%%%With new functionality in the app, this function may become obsolete.
%% Inputs: 
%%%     sensorplacement: struct with six values indicating sensor placement 
%%%                     for each segment of test data.
%% Outputs:

function Rename_SensorFiles(sensorplacement)

%%% Variables
Name{1} = sensorplacement.LF;
Name{2} = sensorplacement.LS;
Name{3} = sensorplacement.LT;
Name{4} = sensorplacement.RF;
Name{5} = sensorplacement.RS;
Name{6} = sensorplacement.RT;

%checkfiles = cell(1,length(names_cell));

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
    
end
