%% Joint Rate Determination
%%% Note: Nothing in this script will need to be updated.

%% Initialize Variables
sample_offset = 0;

legs_num = length(fieldnames(ActData.datasegment(1)));
legs_name = fieldnames(ActData.datasegment(1));
num_dataseg = length(ActData.datasegment);

%% Decouple Joint Rates

for set=1:num_dataseg
    for leg=1:legs_num
        rates.datasegment(set).(legs_name{leg}).Time = ActData.datasegment(set).(legs_name{leg}).Time; 
        
        GyroThighTemp = SensData.datasegment(set).(legs_name{leg}).Thigh.Gyro;
        GyroShinTemp = SensData.datasegment(set).(legs_name{leg}).Shin.Gyro;
        GyroFootTemp = SensData.datasegment(set).(legs_name{leg}).Foot.Gyro;
        [n, ~] = size(GyroThighTemp);
        
    %%% Determine Hip Rates
        gyro_T = GyroThighTemp(1+sample_offset:end,:);
        for k=2:n
            rates.dataset(set).(legs_name{leg}).HF(k,1) = dot(Axes.(legs_name{leg}).HF,gyro_T(k,2:4)');
            rates.dataset(set).(legs_name{leg}).HA(k,1) = dot(Axes.(legs_name{leg}).HA,gyro_T(k,2:4)');
            rates.dataset(set).(legs_name{leg}).HR(k,1) = dot(Axes.(legs_name{leg}).HR,gyro_T(k,2:4)');
        end
        
    %%% Determine Knee Rates
        v_KF = Axes.(legs_name{leg}).KF; %knee flexion joint axis
        
        gyro_T = GyroThighTemp(1+sample_offset:end,:);
        gyro_S = GyroShinTemp(1:end-sample_offset,:);
    
        %%% Gather Gravity Vectors when at Rest (equal weight given between
        %%% gravity vectors and axes of rotation)
        Thigh = [Axes.(legs_name{leg}).HF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Thigh.Accel(1,2:4)];
        Shin = [Axes.(legs_name{leg}).KF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Shin.Accel(1+sample_offset,2:4)];
        for k=3:length(ActData.datasegment(set).(legs_name{leg}).PredLabelsT)-2
            if (ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k)==0&&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k+1)==0 ...
                    &&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k-1)==0&&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k+2)==0 ...
                    &&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k-2)==0)
                Thigh = [Thigh;Axes.(legs_name{leg}).HF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Thigh.WinAccel{k}(:,2:4)];  
                Shin = [Shin;Axes.(legs_name{leg}).KF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Shin.WinAccel{k}(:,2:4)];  
            end
        end
        
        %%% Determine Initial Rotation
        H = Thigh'*Shin;
        [U,~,V] = svd(H);
        d = det(V*U');
        R_ST_init = V*[1,0,0;0,1,0;0,0,d]*U';
        
        %%% Decouple Shin Gyro from Thigh Gyro
        t_prev = gyro_S(1,1);
        theta_KF_prev = 0;
        Rv = eye(3);
        for k=2:n
            t = gyro_S(k,1);
            gyro_S_rem(k,1:4) = [gyro_S(k,1),((gyro_S(k,2:4)'-Rv'*R_ST_init*gyro_T(k,2:4)'))']; 
            rates.dataset(set).(legs_name{leg}).KF(k,1) = dot(Axes.(legs_name{leg}).KF,gyro_S_rem(k,2:4)');
            
            theta_dot_KF = rates.dataset(set).(legs_name{leg}).KF(k);
            theta_KF = theta_KF_prev + theta_dot_KF*(t-t_prev);
            Rv_KF = cosd(theta_KF)*eye(3) + (1-cosd(theta_KF))*(v_KF*v_KF')+sind(theta_KF)*[0,-v_KF(3),v_KF(2);v_KF(3),0,-v_KF(1);-v_KF(2),v_KF(1),0];
            Rv = Rv_KF;
           
            %%% Iterate
            theta_KF_prev = theta_KF;
            t_prev = t;
        end
        
    %%% Determine Ankle Rates
        v_AF = Axes.(legs_name{leg}).AF;
        v_AI = Axes.(legs_name{leg}).AI;
        
        gyro_S = GyroShinTemp(1+sample_offset:end,:);
        gyro_F = GyroFootTemp(1:end-sample_offset,:);
        
        %%% Gather Gravity Vectors when at Rest (equal weight given between
        %%% gravity vectors and axes of rotation)
        Shin = [Axes.(legs_name{leg}).KF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Shin.Accel(1,2:4)];
        Foot = [Axes.(legs_name{leg}).AF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Foot.Accel(1+sample_offset,2:4)];
        for k=3:length(ActData.datasegment(set).(legs_name{leg}).PredLabelsT)-3
            if (ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k)==0&&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k+1)==0 ...
                    &&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k-1)==0&&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k+2)==0 ...
                    &&ActData.datasegment(set).(legs_name{leg}).PredLabelsT(k-2)==0)
                Shin = [Shin;Axes.(legs_name{leg}).KF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Shin.WinAccel{k}(:,2:4)];  
                Foot = [Foot;Axes.(legs_name{leg}).AF';SensData.datasegment(set).(append((legs_name{leg}),'Meaned')).Foot.WinAccel{k}(:,2:4)];  
            end
        end
        
        %%% Determine Initial Rotation        
        H = (Shin'*Foot);
        [U,~,V] = svd(H);
        d = det(V*U');
        R_FS_init = V*[1,0,0;0,1,0;0,0,d]*U';
        
        %%% Decouple Foot Gyro from Shin Gyro     
        t_prev = gyro_F(1,1);
        theta_AF_prev = 0;
        theta_AI_prev = 0;
        Rv=eye(3);
        for k=2:length(gyro_F(:,1))
            t = gyro_F(k,1);
            gyro_F_rem(k,1:4) = [gyro_F(k,1),((gyro_F(k,2:4)'-Rv'*R_FS_init*gyro_S(k,2:4)'))'];

            rates.dataset(set).(legs_name{leg}).AF(k,1) = dot(v_AF,gyro_F_rem(k,2:4)');
            theta_dot_AF = rates.dataset(set).(legs_name{leg}).AF(k);
            theta_AF = theta_AF_prev + theta_dot_AF*(t-t_prev);
            Rv_AF = cosd(theta_AF)*eye(3) + (1-cosd(theta_AF))*(v_AF*v_AF')+sind(theta_AF)*[0,-v_AF(3),v_AF(2);v_AF(3),0,-v_AF(1);-v_AF(2),v_AF(1),0];
            
            rates.dataset(set).(legs_name{leg}).AI(k,1) = dot(v_AI,gyro_F_rem(k,2:4)');
            theta_dot_AI = rates.dataset(set).(legs_name{leg}).AI(k);
            theta_AI = theta_AI_prev + theta_dot_AI*(t-t_prev);
            Rv_AI = cosd(theta_AI)*eye(3) + (1-cosd(theta_AI))*(v_AI*v_AI')+sind(theta_AI)*[0,-v_AI(3),v_AI(2);v_AI(3),0,-v_AI(1);-v_AI(2),v_AI(1),0];
            
            Rv =  Rv_AF*Rv_AI;
            
            %%% Iterate
            theta_AF_prev = theta_AF;
            theta_AI_prev = theta_AI;
            t_prev = t;
        end
    end
end
%%
save('JointRates.mat','rates');
clearvars -except Constants Flags SensData ActData Truth rates