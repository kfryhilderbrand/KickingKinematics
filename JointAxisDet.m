%% Joint Axis Detection
%%% Note: Nothing in this script will need to be updated.

%% Initialize Variables
%%% Number of consecutive windows to qualify as an isolated movement (used
%%% to filter out potential movement artifacts)
num_wind = 5;

windowsize = Constants.windowsize;
%%%Check for even windowsize
if mod(windowsize,2)
    windowsize = windowsize+1;
end

Isolated_Thigh = cell(1,length(SensData.datasegment));
Isolated_Shin = cell(1,length(SensData.datasegment));
Isolated_Foot = cell(1,length(SensData.datasegment));

GyroThighAll = [];
GyroShinAll = [];
GyroFootAll = [];

legs_num = length(fieldnames(ActData.datasegment(1)));
legs_name = fieldnames(ActData.datasegment(1));
num_dataseg = length(ActData.datasegment);

%% Gather Isolated Angular Rate Data and Determine Joint Axes
%%% Determines periods of isolated movement for each limb segment and
%%% gathers associated gyroscope data (the isolated angular rate data for
%%% each limb segment). Then uses PCA on isolated angular rate data to
%%% determine joint axes.

for leg=1:legs_num
    for set=1:num_dataseg
        
        %%% Predicted activity for each limb segment
        Foot = ActData.datasegment(set).(legs_name{leg}).Foot.PredLabelsT;
        Shin = ActData.datasegment(set).(legs_name{leg}).Shin.PredLabelsT;
        Thigh = ActData.datasegment(set).(legs_name{leg}).Thigh.PredLabelsT;
        
        %%% Determine number of instances of rest
        NM = find(Foot+Shin+Thigh==0);
        fprintf('Number of windows datasegment %d: %d \n',set,length(Foot))
        fprintf('NM datasegment %d: %d\n\n',set,length(NM))
        
   %%% Isolated Thigh Movement     
        %%% Gathers data when Thigh movement isolated (All sensors active)
        Thigh_ind = find(Thigh==1);
        Shin_ind = find(Shin==1);
        Foot_ind = find(Foot==1);
        Isolated_Thigh{set} = intersect(Thigh_ind, intersect(Shin_ind, Foot_ind));
        
        %%%Find sequences of <num_wind> windows of isolated movements (Thigh)
        changes=diff(Isolated_Thigh{set}');
        change_ind=find([changes inf]>1);
        len=diff([0 change_ind]); %length of the sequences
        ends=cumsum(len); %endpoints of the sequences
        winds = ends(len>=num_wind)-len(len>=num_wind)+1; %Starting window of the sequences that are at least <num_wind> windows long
        winds_len = len(len>=num_wind); %number of windows in each sequence >= <num_wind>
        
        for k = 1:length(winds)
            start_ind = Isolated_Thigh{set}(winds(k));
            for m = 1:winds_len(k)
              GyroThighAll = [GyroThighAll; SensData.datasegment(set).(legs_name{leg}).Thigh.WinGyro{start_ind+m-1}(:,2:4)];
            end
        end
        GyroThighAll = unique(GyroThighAll,'rows','stable');
       
   %%% Isolated Shin Movement 
        %%%Gathers data when Shin movement isolated (Shin & Foot sensors active)
        Thigh_ind = find(Thigh==0);
        Shin_ind = find(Shin==1);
        Foot_ind = find(Foot==1);
        Isolated_Shin{set} = intersect(Thigh_ind, intersect(Shin_ind, Foot_ind));
        
        %%%Find sequences of <num_wind> windows of isolated movements (Shin)
        changes=diff(Isolated_Shin{set}');
        change_ind=find([changes inf]>1);
        len=diff([0 change_ind]); %length of the sequences
        ends=cumsum(len); %endpoints of the sequences
        winds = ends(len>=num_wind)-len(len>=num_wind)+1; %Starting window of the sequences that are at least 3 windows long
        winds_len = len(len>=num_wind); %number of windows in each sequence >= <num_wind>
        
        for k = 1:length(winds)
            start_ind = Isolated_Shin{set}(winds(k));
            for m = 1:winds_len(k)
               GyroShinAll = [GyroShinAll; SensData.datasegment(set).(legs_name{leg}).Shin.WinGyro{start_ind+m-1}(:,2:4)];
            end
        end
        GyroShinAll = unique(GyroShinAll,'rows','stable');        
        
   %%% Isolated Foot Movement 
        %%%Gathers data when Ankle movement isolated (Foot sensor active)
        Thigh_ind = find(Thigh==0);
        Shin_ind = find(Shin==0);
        Foot_ind = find(Foot==1);
        Isolated_Foot{set} = intersect(Thigh_ind, intersect(Shin_ind, Foot_ind));
  
        %%%Find sequences of <num_wind> windows of isolated movements (Foot)
        changes=diff(Isolated_Foot{set}');
        change_ind=find([changes inf]>1);
        len=diff([0 change_ind]); %length of the sequences
        ends=cumsum(len); %endpoints of the sequences
        winds = ends(len>=num_wind)-len(len>=num_wind)+1; %Starting window of the sequences that are at least 3 windows long
        winds_len = len(len>=num_wind); %number of windows in each sequence >= <num_wind>
        
        for k = 1:length(winds)
            start_ind = Isolated_Foot{set}(winds(k));
            for m = 1:winds_len(k)
                GyroFootAll = [GyroFootAll; SensData.datasegment(set).(legs_name{leg}).Foot.WinGyro{start_ind+m-1}(:,2:4)];
            end
        end
        GyroFootAll = unique(GyroFootAll,'rows','stable');
    end
    
    %%%PCA method to determine joint axes
    [J1_pca,~,~,~,expl1,~] = pca(GyroThighAll);
    [J2_pca,~,~,~,expl2,~] = pca(GyroShinAll);
    [J3_pca,~,~,~,expl3,~] = pca(GyroFootAll);
    
    Axes.(legs_name{leg}).Hip = J1_pca;
    Axes.(legs_name{leg}).Hipvar = expl1;
    Axes.(legs_name{leg}).Knee = J2_pca;
    Axes.(legs_name{leg}).Kneevar = expl2;
    Axes.(legs_name{leg}).Ankle = J3_pca;
    Axes.(legs_name{leg}).Anklevar = expl3;
    
    
    %%%Assign joint axes
    Axes.(legs_name{leg}).HF = J1_pca(:,1);
    Axes.(legs_name{leg}).HA = J1_pca(:,2);
    Axes.(legs_name{leg}).HR = J1_pca(:,3);
    Axes.(legs_name{leg}).KF = J2_pca(:,1);
    Axes.(legs_name{leg}).AF = J3_pca(:,1);
    Axes.(legs_name{leg}).AI = J3_pca(:,2);
    
    for num_axs=1:3 
        HF_ang = atan2(norm(cross([0,1,0],Axes.(legs_name{leg}).HF)),dot([0,1,0],Axes.(legs_name{leg}).HF));
        if atan2(norm(cross([0,1,0],J1_pca(:,num_axs))),dot([0,1,0],J1_pca(:,num_axs)))<HF_ang
            Axes.(legs_name{leg}).HF = J1_pca(:,num_axs);
        end
        HA_ang = atan2(norm(cross([0,0,1],Axes.(legs_name{leg}).HA)),dot([0,0,1],Axes.(legs_name{leg}).HA));
        if atan2(norm(cross([0,0,1],J1_pca(:,num_axs))),dot([0,0,1],J1_pca(:,num_axs)))<HA_ang
            Axes.(legs_name{leg}).HA = J1_pca(:,num_axs);
        end
        KF_ang = atan2(norm(cross([0,1,0],Axes.(legs_name{leg}).KF)),dot([0,1,0],Axes.(legs_name{leg}).KF));
        if atan2(norm(cross([0,1,0],J2_pca(:,num_axs))),dot([0,1,0],J2_pca(:,num_axs)))<KF_ang
            Axes.(legs_name{leg}).KF = J2_pca(:,num_axs);
        end
        AF_ang = atan2(norm(cross([0,1,0],Axes.(legs_name{leg}).AF)),dot([0,1,0],Axes.(legs_name{leg}).AF));
        if atan2(norm(cross([0,1,0],J3_pca(:,num_axs))),dot([0,1,0],J3_pca(:,num_axs)))<AF_ang
            Axes.(legs_name{leg}).AF = J3_pca(:,num_axs);
        end
        AI_ang = atan2(norm(cross([1,0,0],Axes.(legs_name{leg}).AI)),dot([1,0,0],Axes.(legs_name{leg}).AI));
        if atan2(norm(cross([1,0,0],J3_pca(:,num_axs))),dot([1,0,0],J3_pca(:,num_axs)))<AI_ang
            Axes.(legs_name{leg}).AI = J3_pca(:,num_axs);
        end
    end
    Axes.(legs_name{leg}).HR = cross(Axes.(legs_name{leg}).HF, Axes.(legs_name{leg}).HA);
end

save('Axes.mat','Axes');
clearvars -except Axes SensData ActData Truth Constants Flags