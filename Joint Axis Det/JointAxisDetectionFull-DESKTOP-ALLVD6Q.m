%% Joint axis detection with different combinations of sensor activity
%%% All 3 active, Shin and Foot active, Foot active
%%% Other combinations unlikely: Thigh and Shin active, Thigh and foot active
%%% Thigh active, Shin active

%%% 1 DOF movements (this method assumes that the axis of motion falls
%%% within certain quadrants....i.e. flexion axis will have a positive y
%%% component)
%% Load in ActData and SensData and set constants
clear
load('ActivityData.mat')
load('SensorData.mat')
load('Truth.mat')

ConstantsFlags
windowsize = Constants.windowsize;
%%%Check for even windowsize
if mod(windowsize,2)
    windowsize = windowsize+1;
end
Flag_flip = 0;
varexp = 90;
angle_lim = 15; %any vector more than 15 degrees from a group is not part of the group
group_lim = 2; %how many vectors must be in a group to be a group
num_wind = 5;

% % %% Concatenate data
% close all
% GyroFootR = [];
% GyroShinR = [];
% GyroThighR = [];
% 
% GyroThighRComplete = [];
% GyroShinRComplete = [];
% GyroFootRComplete = [];
% for i=1:length(SensData.datasegment) %%%create vector of gyro data when active
%     [Foot, Shin, Thigh] = IndGMMAct(i,ActData,Truth);
%     %[Foot, Shin, Thigh] = IndGMMActComplete(i,ActData,Truth);
%     
% %     Foot = ActData.datasegment(i).Right.Foot.PredLabelsInd;
% %     Shin = ActData.datasegment(i).Right.Shin.PredLabelsInd;
% %     Thigh = ActData.datasegment(i).Right.Thigh.PredLabelsInd;
% 
%     NM = find(Foot+Shin+Thigh==0);
%     fprintf('Number of windows datasegment %d: %d \n',i,length(Foot))
%     fprintf('NM datasegment %d: %d\n\n',i,length(NM))
% 
%     %%%Gathers data when Thigh movement isolated (All sensors active) for a
%     %%%3 sample window
%     for j=2:length(Thigh)-1
%         if ((Thigh(j)==1&&Shin(j)==1&&Foot(j)==1)...
%                &&(Thigh(j-1)==1&&Shin(j-1)==1&&Foot(j-1)==1)...
%                &&(Thigh(j+1)==1&&Shin(j+1)==1&&Foot(j+1)==1))            
%           GyroThighR = [GyroThighR; [SensData.datasegment(i).Right.Thigh.WinTime(j),...
%               SensData.datasegment(i).Right.Thigh.WinGyro{j}(1+windowsize/2,2:4)]];
%           GyroThighRComplete = [GyroThighRComplete; ...
%               SensData.datasegment(i).Right.Thigh.WinGyro{j}(:,2:4)];
% %           GyroThighRComplete = [GyroThighRComplete; ...
% %                 SensData.datasegment(i).Right.Thigh.Gyro(j,2:4)];
%         end
%     end
%     GyroThighRCompleteUnique = unique(GyroThighRComplete,'rows','stable') ;
% 
%     %%%Gathers data when Shin movement isolated (Shin & Foot sensors active)
%     for j=2:length(Shin)-1
%         if ((Thigh(j)==0&&Shin(j)==1&&Foot(j)==1)...
%               &&(Thigh(j-1)==0&&Shin(j-1)==1&&Foot(j-1)==1)...
%                &&(Thigh(j+1)==0&&Shin(j+1)==1&&Foot(j+1)==1))
%             GyroShinR = [GyroShinR; [SensData.datasegment(i).Right.Shin.WinTime(j),...
%                 SensData.datasegment(i).Right.Shin.WinGyro{j}(1+windowsize/2,2:4)]];
%             GyroShinRComplete = [GyroShinRComplete; ...
%                 SensData.datasegment(i).Right.Shin.WinGyro{j}(:,2:4)];
% %           GyroShinRComplete = [GyroShinRComplete; ...
% %                 SensData.datasegment(i).Right.Shin.Gyro(j,2:4)];
%         end
%     end
%     GyroShinRCompleteUnique = unique(GyroShinRComplete,'rows','stable') ;
% 
%     %%%Gathers data when Foot movement isolated
%     for j=2:length(Foot)-1
%         if ((Thigh(j)==0&&Shin(j)==0&&Foot(j)==1)...
%                 &&(Thigh(j-1)==0&&Shin(j-1)==0&&Foot(j-1)==1)...
%                 &&(Thigh(j+1)==0&&Shin(j+1)==0&&Foot(j+1)==1))
%             GyroFootR = [GyroFootR; [SensData.datasegment(i).Right.Foot.WinTime(j),...
%                 SensData.datasegment(i).Right.Foot.WinGyro{j}(1+windowsize/2,2:4)]];
%             GyroFootRComplete = [GyroFootRComplete; ...
%                 SensData.datasegment(i).Right.Foot.WinGyro{j}(:,2:4)];
% %           GyroFootRComplete = [GyroFootRComplete; ...
% %                 SensData.datasegment(i).Right.Foot.Gyro(j,2:4)];
%         end
%     end
%     GyroFootRCompleteUnique = unique(GyroFootRComplete,'rows','stable') ;
% end
% 
% [J1_pca,~,~,~,explained1,~] = pca(GyroThighRCompleteUnique);
% [J2_pca,~,~,~,explained2,~] = pca(GyroShinRCompleteUnique);
% [J3_pca,~,~,~,explained3,~] = pca(GyroFootRCompleteUnique);
start = 1;
ending = 2;
%%
%%% Concatenate data
Isolated_Thigh = cell(1,length(SensData.datasegment));
Isolated_Shin = cell(1,length(SensData.datasegment));
Isolated_Foot = cell(1,length(SensData.datasegment));

for i=start:ending%1:length(SensData.datasegment) %%%create vector of gyro data when active
 %    [Foot, Shin, Thigh] = IndGMMAct(i,ActData,Truth);
    %[Foot, Shin, Thigh] = IndGMMActComplete(i,ActData,Truth);
%     
%     Foot = ActData.datasegment(i).Right.Foot.PredLabelsInd2;
%     Shin = ActData.datasegment(i).Right.Shin.PredLabelsInd2;
%     Thigh = ActData.datasegment(i).Right.Thigh.PredLabelsInd2;

    Foot = ActData.datasegment(i).Right.Foot.PredLabelsT;
    Shin = ActData.datasegment(i).Right.Shin.PredLabelsT;
    Thigh = ActData.datasegment(i).Right.Thigh.PredLabelsT;

    NM = find(Foot+Shin+Thigh==0);
    fprintf('Number of windows datasegment %d: %d \n',i,length(Foot))
    fprintf('NM datasegment %d: %d\n\n',i,length(NM))

    %%%Gathers data when Thigh movement isolated (All sensors active)
    Thigh_ind = find(Thigh==1);
    Shin_ind = find(Shin==1);
    Foot_ind = find(Foot==1);

    Isolated_Thigh{i} = intersect(Thigh_ind, intersect(Shin_ind, Foot_ind));
       
    %%%Gathers data when Shin movement isolated (Shin & Foot sensors active)
    Thigh_ind = find(Thigh==0);
    Shin_ind = find(Shin==1);
    Foot_ind = find(Foot==1); 

    Isolated_Shin{i} = intersect(Thigh_ind, intersect(Shin_ind, Foot_ind));
    
    %%%Gathers data when Ankle movement isolated (Foot sensor active)
    Thigh_ind = find(Thigh==0);
    Shin_ind = find(Shin==0);
    Foot_ind = find(Foot==1); 

    Isolated_Foot{i} = intersect(Thigh_ind, intersect(Shin_ind, Foot_ind));
    %Isolated_Shin{i} = Isolated_Foot{i};
end
%%
GyroFootR = cell(1,length(SensData.datasegment));
GyroShinR = cell(1,length(SensData.datasegment));
GyroThighR = cell(1,length(SensData.datasegment));

GyroThighRComplete = cell(1,length(SensData.datasegment));
GyroShinRComplete = cell(1,length(SensData.datasegment));
GyroFootRComplete = cell(1,length(SensData.datasegment));

GyroThighRAll = [];
GyroShinRAll = [];
GyroFootRAll = [];

GyroThighRCent = [];

for i=start:ending%1:length(SensData.datasegment) %%%create vector of gyro data when active
    %%%Find sequences of 3 windows of isolated movements
    changes=diff(Isolated_Thigh{i}');
    change_ind=find([changes inf]>1);
    len=diff([0 change_ind]); %length of the sequences
    ends=cumsum(len); %endpoints of the sequences
    winds = ends(len>=num_wind)-len(len>=num_wind)+1; %Starting window of the sequences that are at least 3 windows long
    winds_len = len(len>=num_wind); %number of windows in each sequence >= 3
    
    GyroThighR{i} = cell(1,length(winds));
    GyroThighRComplete{i} = cell(1,length(winds));
    for k = 1:length(winds)
        start_ind = Isolated_Thigh{i}(winds(k));
        GyroThighR{i}{k} = [];
        for m = 1:winds_len(k)
            GyroThighR{i}{k} = [GyroThighR{i}{k}; [SensData.datasegment(i).Right.Thigh.WinTime(start_ind+m-1),...
                SensData.datasegment(i).Right.Thigh.WinGyro{start_ind+m-1}(1+windowsize/2,2:4)]];
            GyroThighRComplete{i}{k} = [GyroThighRComplete{i}{k}; ...
                SensData.datasegment(i).Right.Thigh.WinGyro{start_ind+m-1}(:,2:4)];
            GyroThighRAll = [GyroThighRAll; SensData.datasegment(i).Right.Thigh.WinGyro{start_ind+m-1}(:,2:4)];
        end
        GyroThighRComplete{i}{k} = unique(GyroThighRComplete{i}{k},'rows','stable') ;
    end
    GyroThighRAll = unique(GyroThighRAll,'rows','stable');
end

for i=start:ending%1:length(SensData.datasegment) %%%create vector of gyro data when active
    %%%Find sequences of 3 windows of isolated movements
    changes=diff(Isolated_Shin{i}');
    change_ind=find([changes inf]>1);
    len=diff([0 change_ind]); %length of the sequences
    ends=cumsum(len); %endpoints of the sequences
    winds = ends(len>=num_wind)-len(len>=num_wind)+1; %Starting window of the sequences that are at least 3 windows long
    winds_len = len(len>=num_wind); %number of windows in each sequence >= 3
    
    GyroShinR{i} = cell(1,length(winds));
    GyroShinRComplete{i} = cell(1,length(winds));
    for k = 1:length(winds)
        start_ind = Isolated_Shin{i}(winds(k));
        GyroShinR{i}{k} = [];
        for m = 1:winds_len(k)
            GyroShinR{i}{k} = [GyroShinR{i}{k}; [SensData.datasegment(i).Right.Shin.WinTime(start_ind+m-1),...
                SensData.datasegment(i).Right.Shin.WinGyro{start_ind+m-1}(1+windowsize/2,2:4)]];
            GyroShinRComplete{i}{k} = [GyroShinRComplete{i}{k}; ...
                SensData.datasegment(i).Right.Shin.WinGyro{start_ind+m-1}(:,2:4)];
            GyroShinRAll = [GyroShinRAll; SensData.datasegment(i).Right.Shin.WinGyro{start_ind+m-1}(:,2:4)];
        end
        GyroShinRComplete{i}{k} = unique(GyroShinRComplete{i}{k},'rows','stable') ;
    end
    GyroShinRAll = unique(GyroShinRAll,'rows','stable');
end

for i=start:ending%1:length(SensData.datasegment) %%%create vector of gyro data when active
    %%%Find sequences of 3 windows of isolated movements
    changes=diff(Isolated_Foot{i}');
    change_ind=find([changes inf]>1);
    len=diff([0 change_ind]); %length of the sequences
    ends=cumsum(len); %endpoints of the sequences
    winds = ends(len>=num_wind)-len(len>=num_wind)+1; %Starting window of the sequences that are at least 3 windows long
    winds_len = len(len>=num_wind); %number of windows in each sequence >= 3
    
    GyroFootR{i} = cell(1,length(winds));
    GyroFootRComplete{i} = cell(1,length(winds));
    for k = 1:length(winds)
        start_ind = Isolated_Foot{i}(winds(k));
        GyroFootR{i}{k} = [];
        for m = 1:winds_len(k)
            GyroFootR{i}{k} = [GyroFootR{i}{k}; [SensData.datasegment(i).Right.Foot.WinTime(start_ind+m-1),...
                SensData.datasegment(i).Right.Foot.WinGyro{start_ind+m-1}(1+windowsize/2,2:4)]];
            GyroFootRComplete{i}{k} = [GyroFootRComplete{i}{k}; ...
                SensData.datasegment(i).Right.Foot.WinGyro{start_ind+m-1}(:,2:4)];
            GyroFootRAll = [GyroFootRAll; SensData.datasegment(i).Right.Foot.WinGyro{start_ind+m-1}(:,2:4)];
        end
        GyroFootRComplete{i}{k} = unique(GyroFootRComplete{i}{k},'rows','stable') ;
    end
    GyroFootRAll = unique(GyroFootRAll,'rows','stable');
end
%% PCA method (for summarizing)
[J1_pca,~,~,~,explained1,~] = pca(GyroThighRAll);
[J2_pca,~,~,~,explained2,~] = pca(GyroShinRAll);
[J3_pca,~,~,~,explained3,~] = pca(GyroFootRAll);
clearvars -except J1_pca J2_pca J3_pca explained1 explained2 explained3 SensData Act Data Constants Flags


%% PCA method 
%%%Uses gyroscope data at the center of all active windows. PCA is run on
%%%that data and a component is considered to be a joint axis if the
%%%variance explained exceeds 95%.

%%%%%%%%%%%%%%Add in error function (needs to be able to determine which
%%%%%%%%%%%%%%axis to compare to...
SensAxes.Thigh = cell(1,length(SensData.datasegment));
SensAxes.Shin = cell(1,length(SensData.datasegment));
SensAxes.Foot = cell(1,length(SensData.datasegment));

%%%%% Hip axes
for i = 1:length(SensData.datasegment)
    p = length(GyroThighR{i});
    SensAxes.Thigh{i} = cell(2,p);
    if p==0
        disp('No active thigh data available!')
    else
        for k=1:p
            [J1_pca,~,~,~,explained1,~] = pca(GyroThighRComplete{i}{k});
            
            if explained1(1) > varexp
                SensAxes.Thigh{i}{1,k} = J1_pca(:,1);
                SensAxes.Thigh{i}{2,k} = explained1(1);
            elseif explained1(1) + explained1(2) > varexp
                SensAxes.Thigh{i}{1,k} = J1_pca(:,1:2);
                SensAxes.Thigh{i}{2,k} = explained1(1:2);
            else
                SensAxes.Thigh{i}{1,k} = J1_pca;
                SensAxes.Thigh{i}{2,k} = explained1;
            end
        end
    end
    
    %%Gather vectors from windows 
    direc = [];
    n=1;
    for k=1:p
        if length(SensAxes.Thigh{i}{2,k})==1
            direc = [direc;SensAxes.Thigh{i}{1,k}'];
%        elseif length(SensAxes.Thigh{i}{2,k})==2
%           direc = [direc;SensAxes.Thigh{i}{1,k}(:,1)';SensAxes.Thigh{i}{1,k}(:,1)'];
        end
    end
   
    if i==1
        
        %%% Axes into like groups
        direc_temp = direc;
        other_dir = [];
        [m,~] = size(direc_temp);
        
        while m>0
            direct = direc_temp(1,:); %Sort first direction into first group
            for l = 2:m
                if atan2d(norm(cross(direct(1,:),direc_temp(l,:))), dot(direct(1,:),direc_temp(l,:)))<angle_lim
                    direct = [direct; direc_temp(l,:)];
                elseif atan2d(norm(cross(-direct(1,:),direc_temp(l,:))), dot(-direct(1,:),direc_temp(l,:)))<angle_lim %checks to see if this is the negative version of group 1
                    direct = [direct; -direc_temp(l,:)];
                else
                    other_dir = [other_dir; direc_temp(l,:)];
                end
            end
            [m,~] = size(other_dir);
            direc_temp = other_dir;
            other_dir = [];
            
            [r, ~] = size(direct);
            if r>group_lim
                Dir1{n} = direct;
                AvgDir1{n} = mean(direct)';
                n=n+1;
            end
        end
    else
        %%% Axes into like groups
        direc_temp = direc;
        other_dir = [];
        [m,~] = size(direc_temp);
        
        for q = 1:length(Dir1)
            for l = 1:m
                if atan2d(norm(cross(AvgDir1{q},direc_temp(l,:))), dot(AvgDir1{q},direc_temp(l,:)))<angle_lim
                    Dir1{q} = [Dir1{q}; direc_temp(l,:)];
                elseif atan2d(norm(cross(-AvgDir1{q},direc_temp(l,:))), dot(-AvgDir1{q},direc_temp(l,:)))<angle_lim %checks to see if this is the negative version of group 1
                    Dir1{q} = [Dir1{q}; -direc_temp(l,:)];
                else
                    other_dir = [other_dir; direc_temp(l,:)];
                end
            end
            direc_temp = other_dir;
            [m,~] = size(direc_temp);
            other_dir = [];
        end
        [m,~] = size(other_dir);
        if m>group_lim   %%%%This currently doesn't guarentee that extra vectors are of same group
            Dir1{n} = other_dir;
            AvgDir1{n} = mean(direct)';
            n=n+1;
        end
    end
end

%%%% Knee axes
for i = 1:length(SensData.datasegment)
    p = length(GyroShinR{i});
    SensAxes.Shin{i} = cell(2,p);
    if p==0
        disp('No active shin data available!')
    else
        for k=1:p
            [J2_pca,~,~,~,explained2,~] = pca(GyroShinRComplete{i}{k});
            
            if explained2(1) > varexp
                SensAxes.Shin{i}{1,k} = J2_pca(:,1);
                SensAxes.Shin{i}{2,k} = explained2(1);
            elseif explained2(1) + explained2(2) > varexp
                SensAxes.Shin{i}{1,k} = J2_pca(:,1:2);
                SensAxes.Shin{i}{2,k} = explained2(1:2);
            else
                SensAxes.Shin{i}{1,k} = J2_pca;
                SensAxes.Shin{i}{2,k} = explained2;
            end
        end
    end
    
    %%Gather vectors from windows 
    direc = [];
    n=1;
    for k=1:p
        if length(SensAxes.Shin{i}{2,k})==1
            direc = [direc;SensAxes.Shin{i}{1,k}'];
 %       elseif length(SensAxes.Shin{i}{2,k})==2
 %          direc = [direc;SensAxes.Shin{i}{1,k}(:,1)';SensAxes.Thigh{i}{1,k}(:,1)'];
        end
    end
   
    if i==1
        
        %%% Axes into like groups
        direc_temp = direc;
        other_dir = [];
        [m,~] = size(direc_temp);
        
        while m>0
            direct = direc_temp(1,:); %Sort first direction into first group
            for l = 2:m
                if atan2d(norm(cross(direct(1,:),direc_temp(l,:))), dot(direct(1,:),direc_temp(l,:)))<angle_lim
                    direct = [direct; direc_temp(l,:)];
                elseif atan2d(norm(cross(-direct(1,:),direc_temp(l,:))), dot(-direct(1,:),direc_temp(l,:)))<angle_lim %checks to see if this is the negative version of group 1
                    direct = [direct; -direc_temp(l,:)];
                else
                    other_dir = [other_dir; direc_temp(l,:)];
                end
            end
            [m,~] = size(other_dir);
            direc_temp = other_dir;
            other_dir = [];
            
            [r, ~] = size(direct);
            if r>group_lim
                Dir2{n} = direct;
                AvgDir2{n} = mean(direct)';
                n=n+1;
            end
        end
    else
        %%% Axes into like groups
        direc_temp = direc;
        other_dir = [];
        [m,~] = size(direc_temp);
        
        for q = 1:length(Dir2)
            for l = 1:m
                if atan2d(norm(cross(AvgDir2{q},direc_temp(l,:))), dot(AvgDir2{q},direc_temp(l,:)))<angle_lim
                    Dir2{q} = [Dir2{q}; direc_temp(l,:)];
                elseif atan2d(norm(cross(-AvgDir2{q},direc_temp(l,:))), dot(-AvgDir2{q},direc_temp(l,:)))<angle_lim %checks to see if this is the negative version of group 1
                    Dir2{q} = [Dir2{q}; -direc_temp(l,:)];
                else
                    other_dir = [other_dir; direc_temp(l,:)];
                end
            end
            direc_temp = other_dir;
            [m,~] = size(direc_temp);
            other_dir = [];
        end
        [m,~] = size(other_dir);
        if m>group_lim   %%%%This currently doesn't guarentee that extra vectors are of same group
            Dir2{n} = other_dir;
            AvgDir2{n} = mean(direct)';
            n=n+1;
        end
    end
end

%%%% Ankle axes
for i = 1:length(SensData.datasegment)
    p = length(GyroFootR{i});
    SensAxes.Foot{i} = cell(2,p);
    if p==0
        disp('No active ankle data available!')
    else
        for k=1:p
            [J3_pca,~,~,~,explained3,~] = pca(GyroFootRComplete{i}{k});
            
            if explained3(1) > varexp
                SensAxes.Foot{i}{1,k} = J3_pca(:,1);
                SensAxes.Foot{i}{2,k} = explained3(1);
            elseif explained3(1) + explained3(2) > varexp
                SensAxes.Foot{i}{1,k} = J3_pca(:,1:2);
                SensAxes.Foot{i}{2,k} = explained3(1:2);
            else
                SensAxes.Foot{i}{1,k} = J3_pca;
                SensAxes.Foot{i}{2,k} = explained3;
            end
        end
    end
    
    %%Gather vectors from windows 
    direc = [];
    n=1;
    for k=1:p
        if length(SensAxes.Foot{i}{2,k})==1
            direc = [direc;SensAxes.Foot{i}{1,k}'];
%        elseif length(SensAxes.Foot{i}{2,k})==2
%           direc = [direc;SensAxes.Foot{i}{1,k}(:,1)';SensAxes.Foot{i}{1,k}(:,1)'];
        end
    end
   
    if i==1
        
        %%% Axes into like groups
        direc_temp = direc;
        other_dir = [];
        [m,~] = size(direc_temp);
        
        while m>0
            direct = direc_temp(1,:); %Sort first direction into first group
            for l = 2:m
                if atan2d(norm(cross(direct(1,:),direc_temp(l,:))), dot(direct(1,:),direc_temp(l,:)))<angle_lim
                    direct = [direct; direc_temp(l,:)];
                elseif atan2d(norm(cross(-direct(1,:),direc_temp(l,:))), dot(-direct(1,:),direc_temp(l,:)))<angle_lim %checks to see if this is the negative version of group 1
                    direct = [direct; -direc_temp(l,:)];
                else
                    other_dir = [other_dir; direc_temp(l,:)];
                end
            end
            [m,~] = size(other_dir);
            direc_temp = other_dir;
            other_dir = [];
            
            [r, ~] = size(direct);
            if r>group_lim
                Dir3{n} = direct;
                AvgDir3{n} = mean(direct)';
                n=n+1;
            end
        end
    else
        %%% Axes into like groups
        direc_temp = direc;
        other_dir = [];
        [m,~] = size(direc_temp);
        
        for q = 1:length(Dir3)
            for l = 1:m
                if atan2d(norm(cross(AvgDir3{q},direc_temp(l,:))), dot(AvgDir3{q},direc_temp(l,:)))<angle_lim
                    Dir3{q} = [Dir3{q}; direc_temp(l,:)];
                elseif atan2d(norm(cross(-AvgDir3{q},direc_temp(l,:))), dot(-AvgDir3{q},direc_temp(l,:)))<angle_lim %checks to see if this is the negative version of group 1
                    Dir3{q} = [Dir3{q}; -direc_temp(l,:)];
                else
                    other_dir = [other_dir; direc_temp(l,:)];
                end
            end
            direc_temp = other_dir;
            [m,~] = size(direc_temp);
            other_dir = [];
        end
        [m,~] = size(other_dir);
        if m>group_lim   %%%%This currently doesn't guarentee that extra vectors are of same group
            Dir3{n} = other_dir;
            AvgDir3{n} = mean(direct)';
            n=n+1;
        end
    end
end

%%
%     %%% Knee axes
%     [n, ~] = size(GyroShinR);
%     if n==0
%         disp('No active shin data available!')
%         %end
%     else
%         %for i=1:n
%         %[J2_pca,~,~,~,explained2,~] = pca(smoothdata(GyroShinR(:,2:4),1));
%         %[J2_pca,~,~,~,explained2,~] = pca(GyroShinR(:,2:4));
%         [J2_pca,~,~,~,explained2,~] = pca(GyroShinRCompleteUnique);
%         
%         if explained2(1) > varexp
%             SensAxes.Shin{1} = J2_pca(:,1);
%         else
%             SensAxes.Shin{1} = J2_pca(:,1);
%             disp('Error! Something is weird with the knee.')
%         end
%     end
%     
%     %%% Ankle axes
%     [n, ~] = size(GyroFootR);
%     if n==0
%         disp('No active foot data available!')
%         %end
%     else
%         %for i=1:n
%         %[J3_pca,~,~,~,explained3,~] = pca(smoothdata(GyroFootR(:,2:4),1));
%         %[J3_pca,~,~,~,explained3,~] = pca(GyroFootR(:,2:4));
%         [J3_pca,~,~,~,explained3,~] = pca(GyroFootRCompleteUnique);
%         if explained3(1) > varexp
%             SensAxes.Foot{1} = J3_pca(:,1);
%         elseif explained3(1) + explained3(2) > varexp
%             SensAxes.Foot{1} = J3_pca(:,1);
%             SensAxes.Foot{2} = J3_pca(:,2);
%         else
%             disp('Error! Something is weird with the foot.')
%         end
%     end
% end

%% PCA method
%%%Uses gyroscope data at the center of all active windows. PCA is run on
%%%that data and a component is considered to be a joint axis if the
%%%variance explained exceeds 95%. 

%%%%%%%%%%%%%%Add in error function (needs to be able to determine which
%%%%%%%%%%%%%%axis to compare to...

%%% Hip axes
[n, ~] = size(GyroThighR);
if n==0
    disp('No active thigh data available!')
%end
else
%for i=1:n
    %[J1_pca,~,~,~,explained1,~] = pca(smoothdata(GyroThighR(:,2:4),1));
    %[J1_pca,~,~,~,explained1,~] = pca(GyroThighR(:,2:4));
    [J1_pca,~,~,~,explained1,~] = pca(GyroThighRCompleteUnique);

    if explained1(1) > varexp
        SensAxes.Thigh{1} = J1_pca(:,1);
    elseif explained1(1) + explained1(2) > varexp
        SensAxes.Thigh{1} = J1_pca(:,1);
        SensAxes.Thigh{2} = J1_pca(:,2);
    else
        SensAxes.Thigh{1} = J1_pca(:,1);
        SensAxes.Thigh{2} = J1_pca(:,2);
        SensAxes.Thigh{3} = J1_pca(:,3);
    end
end

%%% Knee axes
[n, ~] = size(GyroShinR);
if n==0
    disp('No active shin data available!')
    %end
else
    %for i=1:n
    %[J2_pca,~,~,~,explained2,~] = pca(smoothdata(GyroShinR(:,2:4),1));
    %[J2_pca,~,~,~,explained2,~] = pca(GyroShinR(:,2:4));
    [J2_pca,~,~,~,explained2,~] = pca(GyroShinRCompleteUnique);

    if explained2(1) > varexp
        SensAxes.Shin{1} = J2_pca(:,1);
    else
        SensAxes.Shin{1} = J2_pca(:,1);
        disp('Error! Something is weird with the knee.')
    end
end

%%% Ankle axes
[n, ~] = size(GyroFootR);
if n==0
    disp('No active foot data available!')
%end
else
%for i=1:n
    %[J3_pca,~,~,~,explained3,~] = pca(smoothdata(GyroFootR(:,2:4),1));
    %[J3_pca,~,~,~,explained3,~] = pca(GyroFootR(:,2:4));
    [J3_pca,~,~,~,explained3,~] = pca(GyroFootRCompleteUnique);
    if explained3(1) > varexp
        SensAxes.Foot{1} = J3_pca(:,1);
    elseif explained3(1) + explained3(2) > varexp
        SensAxes.Foot{1} = J3_pca(:,1);
        SensAxes.Foot{2} = J3_pca(:,2);
    else
        disp('Error! Something is weird with the foot.')
    end
end
%%
% init = 0;
% iterations = 100;
% 
% [j1, j2] = JointAxis2Sens(iterations, init, GyroThighREM, GyroShinREM);
% [j3, j4] = JointAxis2Sens(iterations, init, GyroShinREM, GyroFootREM);

%% Error calculation (NAO)
load('CalAxesAll.mat')
%%%Hip
for i=1:length(SensAxes.Thigh)
    switch i
        case 1
            e_temp = norm(SensAxes.Thigh{i}-CalAxesAll.HipFlex);
            Error.Thigh{i} = e_temp;
            angle_temp = atan2d(norm(cross(SensAxes.Thigh{i},CalAxesAll.HipFlex)), dot(SensAxes.Thigh{i},CalAxesAll.HipFlex));
            AngleError.Thigh{i} = angle_temp;
        case 2
            e_temp = norm(SensAxes.Thigh{i}-CalAxesAll.HipAbd);
            Error.Thigh{i} = e_temp;
            angle_temp = atan2d(norm(cross(SensAxes.Thigh{i},CalAxesAll.HipAbd)), dot(SensAxes.Thigh{i},CalAxesAll.HipAbd));
            AngleError.Thigh{i} = angle_temp;
    end
end

%%%Knee
for i=1:length(SensAxes.Shin)
    e_temp = norm(SensAxes.Shin{i}-CalAxesAll.KneeFlex);
    Error.Shin{i} = e_temp;
    angle_temp = atan2d(norm(cross(SensAxes.Shin{i},CalAxesAll.KneeFlex)), dot(SensAxes.Shin{i},CalAxesAll.KneeFlex));
    AngleError.Shin{i} = angle_temp;
end

%%%Ankle
for i=1:length(SensAxes.Foot)
    switch i
        case 1
            e_temp = norm(SensAxes.Foot{i}-CalAxesAll.AnkleFlex);
            Error.Foot{i} = e_temp;
            angle_temp = atan2d(norm(cross(SensAxes.Foot{i},CalAxesAll.AnkleFlex)), dot(SensAxes.Foot{i},CalAxesAll.AnkleFlex));
            AngleError.Foot{i} = angle_temp;
        case 2
            e_temp = norm(SensAxes.Foot{i}-CalAxesAll.AnkleInver);
            Error.Foot{i} = e_temp;
            angle_temp = atan2d(norm(cross(SensAxes.Foot{i},CalAxesAll.AnkleInver)), dot(SensAxes.Foot{i},CalAxesAll.AnkleInver));
            AngleError.Foot{i} = angle_temp;
    end
end
%CalculatedAxes.

%% Joint rates
[n, ~] = size(GyroFootR);
for k=1:n
    %ind=1:length(KickingData.Resamp{3}{2}(:,1))
% g1 = KickingData.Resamp{3}{2}(ind+k-1,2:4)';
% g2 = KickingData.Resamp{2}{2}(ind+k-1,2:4)';
% g3 = KickingData.Resamp{1}{2}(ind+k-1,2:4)';

% g1 = GyroThighR(:,2:4);
% g2 = GyroShinR(:,2:4);
g3 = GyroFootR(k,2:4);

% a1=cross(j1,g1);
% a2=cross(j2,g2);
% a3=cross(j3,g2);

%%During activity, these should all have the same sign...
% b1(k)=dot(j1,g1);
% b2(k)=dot(j2,g2);
% b3(k)=dot(j3,g3);

b1(k,1)=dot(J3_pca(:,1),g3);
b2(k,1)=dot(J3_pca(:,2),g3);
b3(k,1)=dot(J3_pca(:,3),g3);

B1(k,1)=dot(Axes.Foot(2,:)',g3);

end




%% LEFT Concatenate data
close all
GyroFootL = [];
GyroShinL = [];
GyroThighL = [];

i=d;
%for i=1:length(SensData.datasegment) %%%create vector of gyro data when active
   %%%Gathers data when Thigh movement isolated
    for j=2:length(ActData.datasegment(i).Left.Thigh.PredLabels)-1
        if (ActData.datasegment(i).Left.Thigh.PredLabels(j)==1&&...
                ActData.datasegment(i).Left.Thigh.PredLabels(j-1)==1&&...
                ActData.datasegment(i).Left.Thigh.PredLabels(j+1)==1)
            GyroThighL = [GyroThighL; [SensData.datasegment(i).Left.Thigh.WinTime(j),...
                SensData.datasegment(i).Left.Thigh.WinGyro{j}(1+windowsize/2,2:4)]];
        end
    end 
    
    %%%Gathers data when Shin movement isolated
    for j=2:length(ActData.datasegment(i).Left.Shin.PredLabels)-1
        if ((ActData.datasegment(i).Left.Shin.PredLabels(j)==1&&ActData.datasegment(i).Left.Thigh.PredLabels(j)==0)&&...
        (ActData.datasegment(i).Left.Shin.PredLabels(j-1)==1&&ActData.datasegment(i).Left.Thigh.PredLabels(j-1)==0)&&...
        (ActData.datasegment(i).Left.Shin.PredLabels(j+1)==1&&ActData.datasegment(i).Left.Thigh.PredLabels(j+1)==0))
            GyroShinL = [GyroShinL; [SensData.datasegment(i).Left.Shin.WinTime(j),...
                SensData.datasegment(i).Left.Shin.WinGyro{j}(1+windowsize/2,2:4)]];
        end       
    end   
    
    %%%Gathers data when Foot movement isolated
    for j=2:length(ActData.datasegment(i).Left.Foot.PredLabels)-1
        if ((ActData.datasegment(i).Left.Foot.PredLabels(j)==1&&ActData.datasegment(i).Left.Shin.PredLabels(j)==0&&ActData.datasegment(i).Left.Thigh.PredLabels(j)==0)&&...
        (ActData.datasegment(i).Left.Foot.PredLabels(j-1)==1&&ActData.datasegment(i).Left.Shin.PredLabels(j-1)==0&&ActData.datasegment(i).Left.Thigh.PredLabels(j-1)==0)&&...
        (ActData.datasegment(i).Left.Foot.PredLabels(j+1)==1&&ActData.datasegment(i).Left.Shin.PredLabels(j+1)==0&&ActData.datasegment(i).Left.Thigh.PredLabels(j+1)==0))
            GyroFootL = [GyroFootL; [SensData.datasegment(i).Left.Foot.WinTime(j),...
                SensData.datasegment(i).Left.Foot.WinGyro{j}(1+windowsize/2,2:4)]];
        end
    end
%end
 CheckShin.datasegment(i).Left.DateTime = [string(datetime(GyroShinL(:,1), 'convertfrom','posixtime')),GyroShinL(:,1)-fix(GyroShinL(:,1))];
 CheckThigh.datasegment(i).Left.DateTime = [string(datetime(GyroThighL(:,1), 'convertfrom','posixtime')),GyroThighL(:,1)-fix(GyroThighL(:,1))];

%% LEFT PCA method
%%%Uses gyroscope data at the center of all active windows. PCA is run on
%%%that data and a component is considered to be a joint axis if the
%%%variance explained exceeds 95%. 

%%%%%%%%%%%%%%Add in error function (needs to be able to determine which
%%%%%%%%%%%%%%axis to compare to...

%%% Hip axes
[n, ~] = size(GyroThighL);
if n==0
    disp('No active thigh data available!')
%end
else
%for i=1:n
    [J1_pca,~,~,~,explained1,~] = pca(GyroThighL(:,2:4));
    if explained1(1) > 95
        SensAxes.Left.Thigh(1) = J1_pca(:,1);
    elseif explained1(1) + explained1(2) > 95
        SensAxes.Left.Thigh(1) = J1_pca(:,1);
        SensAxes.Left.Thigh(2) = J1_pca(:,2);
    else
        SensAxes.Left.Thigh(1) = J1_pca(:,1);
        SensAxes.Left.Thigh(2) = J1_pca(:,2);
        SensAxes.Left.Thigh(3) = J1_pca(:,3);
    end
end

%%% Knee axes
[n, ~] = size(GyroShinL);
if n==0
    disp('No active shin data available!')
%end
else
%for i=1:n
    [J2_pca,~,~,~,explained2,~] = pca(GyroShinL(:,2:4));
    if explained2(1) > 95
        SensAxes.Left.Shin(1) = J2_pca(:,1);
    else
        disp('Error! Something is weird with the knee.')
    end
end

%%% Ankle axes
[n, ~] = size(GyroFootL);
if n==0
    disp('No active foot data available!')
%end
else
%for i=1:n
    [J3_pca,~,~,~,explained3,~] = pca(GyroFootL(:,2:4));
    if explained3(1) > 95
        SensAxes.Left.Foot(1) = J3_pca(:,1);
    elseif explained3(1) + explained3(2) > 95
        SensAxes.Left.Foot(1) = J3_pca(:,1);
        SensAxes.Left.Foot(2) = J3_pca(:,2);
    else
        disp('Error! Something is weird with the foot.')
    end
end


%% Concatenate data
%%% Creates concatenation of all sensor data within each sensor when sensor
%%% is active
% close all
% GyroFootR = [];
% GyroShinR = [];
% GyroShinR2 = [];
% GyroThighR = [];
% GyroThighR2 = [];
% i=n;
% %for i=1:length(SensData.datasegment) %%%create vector of gyro data when active
%    %%%Gathers data when Thigh movement isolated
%     for j=1:length(ActData.datasegment(i).Right.Thigh.PredLabels)
%         if ActData.datasegment(i).Right.Thigh.PredLabels(j)==1
%             GyroThighR = [GyroThighR; [SensData.datasegment(i).Right.Thigh.WinTime(j),...
%                 SensData.datasegment(i).Right.Thigh.WinGyro{j}(1+windowsize/2,2:4)]];
%         end
%         if (ActData.datasegment(i).Right.Thigh.PredLabels(j)==1&&ActData.datasegment(i).Right.Shin.PredLabels(j)==1)
%             GyroThighR2 = [GyroThighR2; [SensData.datasegment(i).Right.Thigh.WinTime(j),...
%                 SensData.datasegment(i).Right.Thigh.WinGyro{j}(1+windowsize/2,2:4)]];
%         end
%     end 
%     
%     %%%Gathers data when Shin movement isolated
%     for j=1:length(ActData.datasegment(i).Right.Shin.PredLabels)
%         if (ActData.datasegment(i).Right.Shin.PredLabels(j)==1&&ActData.datasegment(i).Right.Thigh.PredLabels(j)==0)
%             GyroShinR = [GyroShinR; [SensData.datasegment(i).Right.Shin.WinTime(j),...
%                 SensData.datasegment(i).Right.Shin.WinGyro{j}(1+windowsize/2,2:4)]];
%         end
%         if (ActData.datasegment(i).Right.Shin.PredLabels(j)==1&&ActData.datasegment(i).Right.Thigh.PredLabels(j)==1)
%             GyroShinR2 = [GyroShinR2; [SensData.datasegment(i).Right.Shin.WinTime(j),...
%                 SensData.datasegment(i).Right.Shin.WinGyro{j}(1+windowsize/2,2:4)]];
%         end        
%     end   
%     
%     %%%Gathers data when Foot movement isolated
%     for j=1:length(ActData.datasegment(i).Right.Foot.PredLabels)
%         if (ActData.datasegment(i).Right.Foot.PredLabels(j)==1 ...
%             &&ActData.datasegment(i).Right.Shin.PredLabels(j)==0&&ActData.datasegment(i).Right.Thigh.PredLabels(j)==0)
%             GyroFootR = [GyroFootR; [SensData.datasegment(i).Right.Foot.WinTime(j),...
%                 SensData.datasegment(i).Right.Foot.WinGyro{j}(1+windowsize/2,2:4)]];
%         end
%     end
% %end

%%
i=d;
%for i=1:length(SensData.datasegment)
    %%%Foot
%     ActData.datasegment(i).Left.Foot.PredLabels = ThreshPred(ActData.datasegment(i).Left.Foot.SHOD,...
%         ActData.datasegment(i).Left.Thresh);
    temp.Foot = ThreshPred(ActData.datasegment(i).Right.Foot.SHOD,...
        ActData.datasegment(i).Right.Thresh_Ind);
% % %     GMM = fitgmdist(ActData.datasegment(i).Right.Foot.SHOD,2);
% % %     temp.FootGMM = cluster(GMM,ActData.datasegment(i).Right.Foot.SHOD)-1;

    %%%Shin
 %    ActData.datasegment(i).Left.Shin.PredLabels = ThreshPred(ActData.datasegment(i).Left.Shin.SHOD,...
 %        ActData.datasegment(i).Left.Thresh);
    temp.Shin = ThreshPred(ActData.datasegment(i).Right.Shin.SHOD,...
        ActData.datasegment(i).Right.Thresh_Ind); 
    
    %%%Thigh
 %    ActData.datasegment(i).Left.Thigh.PredLabels = ThreshPred(ActData.datasegment(i).Left.Thigh.SHOD,...
%         ActData.datasegment(i).Left.Thresh);
    temp.Thigh = ThreshPred(ActData.datasegment(i).Right.Thigh.SHOD,...
        ActData.datasegment(i).Right.Thresh_Ind); 
%end
sum(temp.Foot)
sum(ActData.datasegment(d).Right.Foot.PredLabels)

ShinTruth = [zeros(37000,1);Truth.datasegment(d).Right.WinTruth(37001:end,1)];

cpFoot = classperf(Truth.datasegment(d).Right.WinTruth,ActData.datasegment(d).Right.Foot.PredLabels);
cpFoot2 = classperf(Truth.datasegment(d).Right.WinTruth,temp.Foot);

cpShin = classperf(ShinTruth,ActData.datasegment(d).Right.Shin.PredLabels);
cpShin2 = classperf(ShinTruth,temp.Shin);
%%
%j2 = JointAxis1Sens(500, 0, GyroShinR, 0, [0,1,0]);
[j1, j2] = JointAxis2Sens(500, 0, GyroThighR2, GyroShinR2);