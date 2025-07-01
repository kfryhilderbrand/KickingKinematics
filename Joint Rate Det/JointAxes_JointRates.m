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
% varexp = 90;
% angle_lim = 15; %any vector more than 15 degrees from a group is not part of the group
% group_lim = 2; %how many vectors must be in a group to be a group
 num_wind = 5;
%%
%%% Concatenate data
Isolated_Thigh = cell(1,length(SensData.datasegment));
Isolated_Shin = cell(1,length(SensData.datasegment));
Isolated_Foot = cell(1,length(SensData.datasegment));

GyroFoot= cell(1,length(SensData.datasegment));
GyroShin = cell(1,length(SensData.datasegment));
GyroThigh = cell(1,length(SensData.datasegment));

GyroThighComplete = cell(1,length(SensData.datasegment));
GyroShinComplete = cell(1,length(SensData.datasegment));
GyroFootComplete = cell(1,length(SensData.datasegment));

GyroThighAll = [];
GyroShinAll = [];
GyroFootAll = [];

legs_num = length(fieldnames(ActData.datasegment(1)));
legs_name = fieldnames(ActData.datasegment(1));

for leg=1:legs_num
    for set=1:length(ActData.datasegment)
        
        %%%create vector of gyro data when active
        Foot = ActData.datasegment(set).(legs_name{leg}).Foot.PredLabelsT;
        Shin = ActData.datasegment(set).(legs_name{leg}).Shin.PredLabelsT;
        Thigh = ActData.datasegment(set).(legs_name{leg}).Thigh.PredLabelsT;
        
        NM = find(Foot+Shin+Thigh==0);
        fprintf('Number of windows datasegment %d: %d \n',set,length(Foot))
        fprintf('NM datasegment %d: %d\n\n',set,length(NM))
        
        %%%Gathers data when Thigh movement isolated (All sensors active)
        Thigh_ind = find(Thigh==1);
        Shin_ind = find(Shin==1);
        Foot_ind = find(Foot==1);
        
        Isolated_Thigh{set} = intersect(Thigh_ind, intersect(Shin_ind, Foot_ind));
        
        %%%Gathers data when Shin movement isolated (Shin & Foot sensors active)
        Thigh_ind = find(Thigh==0);
        Shin_ind = find(Shin==1);
        Foot_ind = find(Foot==1);
        
        Isolated_Shin{set} = intersect(Thigh_ind, intersect(Shin_ind, Foot_ind));
        
        %%%Gathers data when Ankle movement isolated (Foot sensor active)
        Thigh_ind = find(Thigh==0);
        Shin_ind = find(Shin==0);
        Foot_ind = find(Foot==1);
        
        Isolated_Foot{set} = intersect(Thigh_ind, intersect(Shin_ind, Foot_ind));
        
        %%%Find sequences of 3 windows of isolated movements (Thigh)
        changes=diff(Isolated_Thigh{set}');
        change_ind=find([changes inf]>1);
        len=diff([0 change_ind]); %length of the sequences
        ends=cumsum(len); %endpoints of the sequences
        winds = ends(len>=num_wind)-len(len>=num_wind)+1; %Starting window of the sequences that are at least 3 windows long
        winds_len = len(len>=num_wind); %number of windows in each sequence >= 3
        
        GyroThigh{set} = cell(1,length(winds));
        GyroThighComplete{set} = cell(1,length(winds));
        for k = 1:length(winds)
            start_ind = Isolated_Thigh{set}(winds(k));
            GyroThigh{set}{k} = [];
            for m = 1:winds_len(k)
                GyroThigh{set}{k} = [GyroThigh{set}{k}; [SensData.datasegment(set).(legs_name{leg}).Thigh.WinTime(start_ind+m-1),...
                    SensData.datasegment(set).(legs_name{leg}).Thigh.WinGyro{start_ind+m-1}(1+windowsize/2,2:4)]];
                GyroThighComplete{set}{k} = [GyroThighComplete{set}{k}; ...
                    SensData.datasegment(set).(legs_name{leg}).Thigh.WinGyro{start_ind+m-1}(:,2:4)];
                GyroThighAll = [GyroThighAll; SensData.datasegment(set).(legs_name{leg}).Thigh.WinGyro{start_ind+m-1}(:,2:4)];
            end
            GyroThighComplete{set}{k} = unique(GyroThighComplete{set}{k},'rows','stable') ;
        end
        GyroThighAll = unique(GyroThighAll,'rows','stable');
        
        %%%Find sequences of 3 windows of isolated movements (Shin)
        changes=diff(Isolated_Shin{set}');
        change_ind=find([changes inf]>1);
        len=diff([0 change_ind]); %length of the sequences
        ends=cumsum(len); %endpoints of the sequences
        winds = ends(len>=num_wind)-len(len>=num_wind)+1; %Starting window of the sequences that are at least 3 windows long
        winds_len = len(len>=num_wind); %number of windows in each sequence >= 3
        
        GyroShin{set} = cell(1,length(winds));
        GyroShinComplete{set} = cell(1,length(winds));
        for k = 1:length(winds)
            start_ind = Isolated_Shin{set}(winds(k));
            GyroShin{set}{k} = [];
            for m = 1:winds_len(k)
                GyroShin{set}{k} = [GyroShin{set}{k}; [SensData.datasegment(set).(legs_name{leg}).Shin.WinTime(start_ind+m-1),...
                    SensData.datasegment(set).(legs_name{leg}).Shin.WinGyro{start_ind+m-1}(1+windowsize/2,2:4)]];
                GyroShinComplete{set}{k} = [GyroShinComplete{set}{k}; ...
                    SensData.datasegment(set).(legs_name{leg}).Shin.WinGyro{start_ind+m-1}(:,2:4)];
                GyroShinAll = [GyroShinAll; SensData.datasegment(set).(legs_name{leg}).Shin.WinGyro{start_ind+m-1}(:,2:4)];
            end
            GyroShinComplete{set}{k} = unique(GyroShinComplete{set}{k},'rows','stable') ;
        end
        GyroShinAll = unique(GyroShinAll,'rows','stable');
        
        %%%Find sequences of 3 windows of isolated movements (Foot)
        changes=diff(Isolated_Foot{set}');
        change_ind=find([changes inf]>1);
        len=diff([0 change_ind]); %length of the sequences
        ends=cumsum(len); %endpoints of the sequences
        winds = ends(len>=num_wind)-len(len>=num_wind)+1; %Starting window of the sequences that are at least 3 windows long
        winds_len = len(len>=num_wind); %number of windows in each sequence >= 5
        
        GyroFoot{set} = cell(1,length(winds));
        GyroFootComplete{set} = cell(1,length(winds));
        for k = 1:length(winds)
            start_ind = Isolated_Foot{set}(winds(k));
            GyroFoot{set}{k} = [];
            for m = 1:winds_len(k)
                GyroFoot{set}{k} = [GyroFoot{set}{k}; [SensData.datasegment(set).(legs_name{leg}).Foot.WinTime(start_ind+m-1),...
                    SensData.datasegment(set).(legs_name{leg}).Foot.WinGyro{start_ind+m-1}(1+windowsize/2,2:4)]];
                GyroFootComplete{set}{k} = [GyroFootComplete{set}{k}; ...
                    SensData.datasegment(set).(legs_name{leg}).Foot.WinGyro{start_ind+m-1}(:,2:4)];
                GyroFootAll = [GyroFootAll; SensData.datasegment(set).(legs_name{leg}).Foot.WinGyro{start_ind+m-1}(:,2:4)];
            end
            GyroFootComplete{set}{k} = unique(GyroFootComplete{set}{k},'rows','stable') ;
        end
        GyroFootAll = unique(GyroFootAll,'rows','stable');
    end
    %%%PCA method (for summarizing)
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

clearvars -except Axes SensData ActData Constants Flags

%%
% clear 
% % load('SensorData.mat','SensData')
% % load('ActivityData.mat')
%load('Axes_Full.mat')    %%%%%%%%%
%Axes.HR = cross(Axes.HF,Axes.HA);%%%%%%%%%%%
%Axes.Right = Axes;%%%%%%%%%%%%%%%%%
sample_offset = 0;
%rates_time = SensData.datasegment(set).Right.Thigh.Gyro(:,1)-SensData.datasegment(set).Right.Thigh.Gyro(1,1);

%%%%Decouple Rates
for set=1:length(ActData.datasegment)
    legs_num = length(fieldnames(ActData.datasegment(set)));
    legs_name = fieldnames(ActData.datasegment(set));
    for leg=1:legs_num %%%%%%%%%%5
        GyroThighTemp = SensData.datasegment(set).(legs_name{leg}).Thigh.Gyro;
        GyroShinTemp = SensData.datasegment(set).(legs_name{leg}).Shin.Gyro;
        GyroFootTemp = SensData.datasegment(set).(legs_name{leg}).Foot.Gyro;
        [n, ~] = size(GyroThighTemp);
        
        gyro_T = GyroThighTemp(1+sample_offset:end,:);
        gyro_S = GyroShinTemp(1:end-sample_offset,:);
        
        %%% Hip Rates
        for k=2:n
            rates.dataset(set).(legs_name{leg}).HF(k,1) = dot(Axes.(legs_name{leg}).HF,gyro_T(k,2:4)');
            rates.dataset(set).(legs_name{leg}).HA(k,1) = dot(Axes.(legs_name{leg}).HA,gyro_T(k,2:4)');
            rates.dataset(set).(legs_name{leg}).HR(k,1) = dot(Axes.(legs_name{leg}).HR,gyro_T(k,2:4)');
        end
        
        %%% Knee Rates
        
        %%%Gather Gravity vectors when at rest (equal weight between gravity
        %%%vectors and axes of rotation
        Thigh = [Axes.(legs_name{leg}).HF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Thigh.Accel(1,2:4)];
        Shin = [Axes.(legs_name{leg}).KF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Shin.Accel(1+sample_offset,2:4)];
        for k=3:length(ActData.datasegment(set).(legs_name{leg}).PredLabelsT)-2
            if (ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k)==0&&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k+1)==0 ...
                    &&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k-1)==0&&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k+2)==0 ...
                    &&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k-2)==0)
                Thigh = [Thigh;Axes.(legs_name{leg}).HF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Thigh.WinAccel{k}(:,2:4)];  %SensData.datasegment(set).(legs_name{leg}).Shin.Accel(i,2:4)];
                Shin = [Shin;Axes.(legs_name{leg}).KF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Shin.WinAccel{k}(:,2:4)];  %SensData.datasegment(set).(legs_name{leg}).Foot.Accel(i+sample_offset,2:4)];
            end
        end
        
        %%%Initial Rotation
        H = Thigh'*Shin;
        [U,~,V] = svd(H);
        d = det(V*U');
        R_ST_init = V*[1,0,0;0,1,0;0,0,d]*U';
        
        v_KF = Axes.(legs_name{leg}).KF;
        
        t_prev = gyro_S(1,1);
        theta_KF_prev = 0;
        Rv = eye(3);
        for k=2:n
            t = gyro_S(k,1);
                                % gyro_S_rem(k,1:4) = [gyro_S(k,1),(R_ST_init*(R_ST_init'*gyro_S(k,2:4)'-gyro_T(k,2:4)'))'];
            gyro_S_rem(k,1:4) = [gyro_S(k,1),((gyro_S(k,2:4)'-Rv'*R_ST_init*gyro_T(k,2:4)'))']; 
            %gyro_S_rem(k,1:4) = [gyro_S(k,1), gyro_S(k,2:4)];
            rates.dataset(set).(legs_name{leg}).KF(k,1) = dot(Axes.(legs_name{leg}).KF,gyro_S_rem(k,2:4)');
            
            theta_dot_KF = rates.dataset(set).(legs_name{leg}).KF(k);
            theta_KF = theta_KF_prev + theta_dot_KF*(t-t_prev);
            Rv_KF = cosd(theta_KF)*eye(3) + (1-cosd(theta_KF))*(v_KF*v_KF')+sind(theta_KF)*[0,-v_KF(3),v_KF(2);v_KF(3),0,-v_KF(1);-v_KF(2),v_KF(1),0];
            Rv = Rv_KF;
           
            
            %Iterate
            %angle_AF = [angle_AF; theta_AF];
            theta_KF_prev = theta_KF;
            t_prev = t;
        end
        
        %%% Ankle Rates
        
        %%%Gather Gravity vectors when at rest (equal weight between gravity
        %%%vectors and axes of rotation
        Shin = [Axes.(legs_name{leg}).KF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Shin.Accel(1,2:4)];
        Foot = [Axes.(legs_name{leg}).AF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Foot.Accel(1+sample_offset,2:4)];
        for k=4:length(ActData.datasegment(set).(legs_name{leg}).PredLabelsT)-3
            if (ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k)==0&&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k+1)==0 ...
                    &&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k-1)==0&&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k+2)==0 ...
                    &&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k-2)==0&&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k+3)==0 ...
                    &&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k-3))
                Shin = [Shin;Axes.(legs_name{leg}).KF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Shin.WinAccel{k}(:,2:4)];  %SensData.datasegment(set).(legs_name{leg}).Shin.Accel(i,2:4)];
                Foot = [Foot;Axes.(legs_name{leg}).AF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Foot.WinAccel{k}(:,2:4)];  %SensData.datasegment(set).(legs_name{leg}).Foot.Accel(i+sample_offset,2:4)];
            end
        end
        
        %%%Initial Rotation
        H = (Shin'*Foot);
        [U,~,V] = svd(H);
        d = det(V*U');
        R_FS_init = V*[1,0,0;0,1,0;0,0,d]*U';
        
      
        gyro_S = GyroShinTemp(1+sample_offset:end,:);
        gyro_F = GyroFootTemp(1:end-sample_offset,:);
        
        
        v_AF = Axes.(legs_name{leg}).AF;
        v_AI = Axes.(legs_name{leg}).AI;
        t_prev = gyro_F(1,1);
        
        theta_AF_prev = 0;
        %angle_AF = [];
        theta_AI_prev = 0;
        %angle_AI = [];
        Rv=eye(3);
        for k=2:length(gyro_F(:,1))
            t = gyro_F(k,1);
                            %gyro_F_rem(k,1:4) = [gyro_F(k,1),(R_FS_init*(R_FS_init'*gyro_F(k,2:4)'-gyro_S(k,2:4)'))'];
            gyro_F_rem(k,1:4) = [gyro_F(k,1),((gyro_F(k,2:4)'-Rv'*R_FS_init*gyro_S(k,2:4)'))'];
            %gyro_F_rem(k,1:4) = [gyro_F(k,1), gyro_F(k,2:4)];

            rates.dataset(set).(legs_name{leg}).AF(k,1) = dot(v_AF,gyro_F_rem(k,2:4)');
            theta_dot_AF = rates.dataset(set).(legs_name{leg}).AF(k);
            theta_AF = theta_AF_prev + theta_dot_AF*(t-t_prev);
            Rv_AF = cosd(theta_AF)*eye(3) + (1-cosd(theta_AF))*(v_AF*v_AF')+sind(theta_AF)*[0,-v_AF(3),v_AF(2);v_AF(3),0,-v_AF(1);-v_AF(2),v_AF(1),0];
            
            rates.dataset(set).(legs_name{leg}).AI(k,1) = dot(v_AI,gyro_F_rem(k,2:4)');
            theta_dot_AI = rates.dataset(set).(legs_name{leg}).AI(k);
            theta_AI = theta_AI_prev + theta_dot_AI*(t-t_prev);
            Rv_AI = cosd(theta_AI)*eye(3) + (1-cosd(theta_AI))*(v_AI*v_AI')+sind(theta_AI)*[0,-v_AI(3),v_AI(2);v_AI(3),0,-v_AI(1);-v_AI(2),v_AI(1),0];
            
            Rv =  Rv_AF*Rv_AI;%(Rv_AI*Rv_AF)';
            
            %Iterate
            %angle_AF = [angle_AF; theta_AF];
            theta_AF_prev = theta_AF;
            %angle_AI = [angle_AI; theta_AI];
            theta_AI_prev = theta_AI;
            t_prev = t;
        end
    end
end
%%
save('JointRates.mat','rates');
clear