clear all

load('SensorData.mat','SensData')
load('Axes_Full.mat')
load('NAOJointRates_set1.mat')
% load('C:\Users\katel\OneDrive - Georgia Institute of Technology\Research\Infant Kicking\__Data\ANALYSIS Data\zICRA_NAO\2021-05-28\2021-05-28\Axes_AnkleCombo.mat')
% load('C:\Users\katel\OneDrive - Georgia Institute of Technology\Research\Infant Kicking\__Data\ANALYSIS Data\zICRA_NAO\2021-05-28\2021-05-28\Axes_HipCombo.mat')
% load('C:\Users\katel\OneDrive - Georgia Institute of Technology\Research\Infant Kicking\__Data\ANALYSIS Data\zICRA_NAO\2021-05-28\2021-05-28\Axes_KF.mat')
set = 1;

GyroThighTemp = SensData.datasegment(set).Right.Thigh.Gyro;
GyroShinTemp = SensData.datasegment(set).Right.Shin.Gyro;
GyroFootTemp = SensData.datasegment(set).Right.Foot.Gyro;

AccelThighTemp = SensData.datasegment(set).Right.Thigh.Accel;
AccelShinTemp = SensData.datasegment(set).Right.Shin.Accel;
[n, ~] = size(GyroThighTemp);


axes_lim = [0 70 -200 200];
%%
 NAOJointRates_resamp(:,1) = GyroThighTemp(:,1);
for i=2:6
    NAOJointRates_resamp(:,i) = interp1(NAOJointRates(:,1), NAOJointRates(:,i), GyroThighTemp(:,1));
end
%%
for k=1:n
 
g1 = GyroThighTemp(k,2:4);
g2 = GyroShinTemp(k,2:4);
g3 = GyroFootTemp(k,2:4);

Axes.HR = cross(Axes.HF,Axes.HA);
b1(k,1)=dot(Axes.HF,g1);
b2(k,1)=dot(Axes.HA,g1);
b3(k,1)=dot(Axes.HR,g1);

c1(k,1)=dot(Axes.KF,g2);
c2(k,1)=dot(Axes.HA,g2);

d1(k,1)=dot(Axes.AF,g3);
d2(k,1)=dot(Axes.AI,g3);

end
t_T = SensData.datasegment(set).Right.Thigh.Gyro(:,1);
t_S = SensData.datasegment(set).Right.Shin.Gyro(:,1);
t_F = SensData.datasegment(set).Right.Foot.Gyro(:,1);

% [c_HF, lags_HF] = xcorr(b1,NAOJointRates_resamp(:,2)*(180/pi),50,'normalized');
% [c_HA, lags_HA] = xcorr(b2,NAOJointRates_resamp(:,3)*(180/pi),50,'normalized');

figure()

subplot(2,1,1)
hold on
plot(t_T-t_T(1),b1)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,2)*(180/pi))
title("Flexion Rates at Thigh Sensor")
ylabel("Degrees/s")
xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

subplot(2,1,2)
hold on
plot(t_T-t_T(1),b2)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,3)*(180/pi))
title("Abduction Rates at Thigh Sensor")
ylabel("Degrees/s")
xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

% figure()
% hold on
% plot(t_S-t_T(1),c1)
% plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,4)*(180/pi))
% title("Flexion Rates at Shin Sensor (Coupled)")
% ylabel("Degrees/s")
% xlabel("Time")
% legend("Calculated","Truth")
% axis(axes_lim)
% grid on
% grid minor
% hold off 
% 
% figure()
% 
% subplot(2,1,1)
% hold on
% plot(t_F-t_T(1),d1)
% plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,5)*(180/pi))
% title("Flexion Rates at Foot Sensor (Coupled)")
% ylabel("Degrees/s")
% xlabel("Time")
% legend("Calculated","Truth")
% axis(axes_lim)
% grid on
% grid minor
% hold off
% 
% subplot(2,1,2)
% hold on
% plot(t_F-t_T(1),d2)
% plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,6)*(180/pi))
% title("Inversion Rates at Foot Sensor (Coupled)")
% ylabel("Degrees/s")
% xlabel("Time")
% legend("Calculated","Truth")
% axis(axes_lim)
% grid on
% grid minor
% hold off
%% Find rotation matrix for Knee

sample_offset = 0;%50;
Thigh = [Axes.HF';SensData.datasegment(set).RightMeaned.Thigh.Accel(1,2:4)];
Shin = [Axes.KF';SensData.datasegment(set).RightMeaned.Shin.Accel(1+sample_offset,2:4)];

load('ActivityData.mat')

for i=3:length(ActData.datasegment(set).Right.PredLabelsT)-2
    if (ActData.datasegment(set).Right.PredLabelsT(i)==0&&ActData.datasegment(set).Right.PredLabelsT(i+1)==0 ...
        &&ActData.datasegment(set).Right.PredLabelsT(i-1)==0&&ActData.datasegment(set).Right.PredLabelsT(i+2)==0 ...
        &&ActData.datasegment(set).Right.PredLabelsT(i-2)==0)
        Thigh = [Thigh;Axes.KF';SensData.datasegment(set).RightMeaned.Thigh.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Right.Shin.Accel(i,2:4)];
        Shin = [Shin;Axes.AF';SensData.datasegment(set).RightMeaned.Shin.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Right.Foot.Accel(i+sample_offset,2:4)];
    end
end

H = Thigh'*Shin;
[U,~,V] = svd(H);
d = det(V*U');
R_ST_init = V*[1,0,0;0,1,0;0,0,d]*U';
%R_ST_init = eye(3);

gyro_T = GyroThighTemp(1+sample_offset:end,:);
gyro_S = GyroShinTemp(1:end-sample_offset,:);

v_KF = Axes.KF;
theta_KF_prev = 0;
t_prev = gyro_S(1,1);
Rv = eye(3);
angle_KF = 0;
rate_KF = 0;
for k=2:length(gyro_S(:,1))
    t = gyro_S(k,1);

    rates.dataset(set).HF(k,1) = dot(Axes.HF,gyro_T(k,2:4)');
    rates.dataset(set).HA(k,1) = dot(Axes.HA,gyro_T(k,2:4)');
    rates.dataset(set).HR(k,1) = dot(Axes.HR,gyro_T(k,2:4)');
    %gyro_S_rem(k,1:4) = [gyro_S(k,1),(gyro_S(k,2:4)'-Rv'*R_ST_init*gyro_T(k,2:4)')'];
    gyro_S_rem(k,1:4) = [gyro_S(k,1),(R_ST_init*(R_ST_init'*gyro_S(k,2:4)'-gyro_T(k,2:4)'))'];
 
    rates.dataset(set).KF(k,1) = dot(Axes.KF,gyro_S_rem(k,2:4)');
    theta_dot_KF = rates.dataset(set).KF(k);
    theta_KF = theta_KF_prev + theta_dot_KF*(t-t_prev);
    Rv = cosd(theta_KF)*eye(3) + (1-cosd(theta_KF))*(v_KF*v_KF')+sind(theta_KF)*[0,-v_KF(3),v_KF(2);v_KF(3),0,-v_KF(1);-v_KF(2),v_KF(1),0];
 
    angle_KF = [angle_KF; theta_KF];
    theta_KF_prev = theta_KF;
    t_prev = t;
end

[c_KFcoup, lags_KFcoup] = xcorr(c1,NAOJointRates_resamp(:,4)*(180/pi),50,'normalized');
[c_KF, lags_KF] = xcorr(rates.dataset(set).KF,NAOJointRates_resamp(:,4)*(180/pi),50,'normalized');

figure()
subplot(2,1,1)
hold on
plot(t_S-t_T(1),c1)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,4)*(180/pi))
title("Flexion Rates at Shin Sensor (Coupled)")
ylabel("Degrees/s")
xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off 

subplot(2,1,2)
hold on
plot(gyro_S(:,1)-t_T(1),rates.dataset(set).KF)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,4)*(180/pi))
title("Flexion Rates at Shin Sensor (Decoupled)")
ylabel("Degrees/s")
xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off 

%% Find rotation matrix for Ankle

sample_offset = 0;%50;

 Shin = [Axes.KF';SensData.datasegment(set).RightMeaned.Shin.Accel(1,2:4)];
 Foot = [Axes.AF';SensData.datasegment(set).RightMeaned.Foot.Accel(1+sample_offset,2:4)];
%Shin = [Axes.KF';(1/norm(SensData.datasegment(set).Right.Shin.WinAccel{1}(:,2:4)))*SensData.datasegment(set).Right.Shin.WinAccel{1}(:,2:4)];
%Foot = [Axes.AF';(1/norm(SensData.datasegment(set).Right.Foot.WinAccel{1}(:,2:4)))*SensData.datasegment(set).Right.Foot.WinAccel{1}(:,2:4)];
for i=4:length(ActData.datasegment(set).Right.PredLabelsT)-3
    if (ActData.datasegment(set).Right.PredLabelsT(i)==0&&ActData.datasegment(set).Right.PredLabelsT(i+1)==0 ...
        &&ActData.datasegment(set).Right.PredLabelsT(i-1)==0&&ActData.datasegment(set).Right.PredLabelsT(i+2)==0 ...
        &&ActData.datasegment(set).Right.PredLabelsT(i-2)==0&&ActData.datasegment(set).Right.PredLabelsT(i+3)==0 ...
        &&ActData.datasegment(set).Right.PredLabelsT(i-3))
        Shin = [Shin;Axes.KF';SensData.datasegment(set).RightMeaned.Shin.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Right.Shin.Accel(i,2:4)];
        Foot = [Foot;Axes.AF';SensData.datasegment(set).RightMeaned.Foot.WinAccel{i}(:,2:4)];  %SensData.datasegment(set).Right.Foot.Accel(i+sample_offset,2:4)];
    end
end

H = (Shin'*Foot);
[U,S,V] = svd(H);
d = det(V*U');
R_FS_init = V*[1,0,0;0,1,0;0,0,d]*U';
%R_FS_init = [0,0,-1;0,1,0;1,0,0];  %goal
%R_FS_init = eye(3);
Rv = eye(3);
%R_FS_init = R_FS_init';

gyro_S = GyroShinTemp(1+sample_offset:end,:);
gyro_F = GyroFootTemp(1:end-sample_offset,:);


v_AF = Axes.AF;
v_AI = Axes.AI;
t_prev = gyro_F(1,1);
Rv = eye(3);
theta_AF_prev = 0;
angle_AF = [];
rate_AF = [];
theta_AI_prev = 0;
angle_AI = [];
rate_AI = [];
for k=2:length(gyro_F(:,1))
    t = gyro_F(k,1);

    %gyro_F_rem(k,1:4) = [gyro_F(k,1),(gyro_F(k,2:4)'-Rv*R_FS_init*gyro_S(k,2:4)')'];
      gyro_F_rem(k,1:4) = [gyro_F(k,1),(R_FS_init*(R_FS_init'*gyro_F(k,2:4)'-gyro_S(k,2:4)'))'];
      
    rates.dataset(set).AF(k,1) = dot(v_AF,gyro_F_rem(k,2:4)');
    theta_dot_AF = rates.dataset(set).AF(k);
    theta_AF = theta_AF_prev + theta_dot_AF*(t-t_prev);
    Rv_AF = cosd(theta_AF)*eye(3) + (1-cosd(theta_AF))*(v_AF*v_AF')+sind(theta_AF)*[0,-v_AF(3),v_AF(2);v_AF(3),0,-v_AF(1);-v_AF(2),v_AF(1),0];
    
    rates.dataset(set).AI(k,1) = dot(v_AI,gyro_F_rem(k,2:4)');
    theta_dot_AI = rates.dataset(set).AI(k);
    theta_AI = theta_AI_prev + theta_dot_AI*(t-t_prev);
    Rv_AI = cosd(theta_AI)*eye(3) + (1-cosd(theta_AI))*(v_AI*v_AI')+sind(theta_AI)*[0,-v_AI(3),v_AI(2);v_AI(3),0,-v_AI(1);-v_AI(2),v_AI(1),0];

    Rv =  (Rv_AI*Rv_AF)';

    %Iterate
    angle_AF = [angle_AF; theta_AF];
    theta_AF_prev = theta_AF;
    angle_AI = [angle_AI; theta_AI];
    theta_AI_prev = theta_AI;    
    t_prev = t;
end

[c_AFcoup, lags_AFcoup] = xcorr(d1,NAOJointRates_resamp(:,5)*(180/pi),50,'normalized');
[c_AIcoup, lags_AIcoup] = xcorr(d2,NAOJointRates_resamp(:,6)*(180/pi),50,'normalized');

[c_AF, lags_AF] = xcorr(rates.dataset(set).AF,NAOJointRates_resamp(:,5)*(180/pi),50,'normalized');
[c_AI, lags_AI] = xcorr(rates.dataset(set).AI,NAOJointRates_resamp(:,6)*(180/pi),50,'normalized');


figure()
subplot(2,1,1)
hold on
plot(t_F-t_T(1),d1)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,5)*(180/pi))
title("Flexion Rates at Foot Sensor (Coupled)")
ylabel("Degrees/s")
xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

subplot(2,1,2)
hold on
plot(gyro_F(:,1)-t_T(1),rates.dataset(set).AF)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,5)*(180/pi))
title("Flexion Rates at Foot Sensor (Decoupled)")
ylabel("Degrees/s")
xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off 

figure()
subplot(2,1,1)
hold on
plot(t_F-t_T(1),d2)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,6)*(180/pi))
title("Inversion Rates at Foot Sensor (Coupled)")
ylabel("Degrees/s")
xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

subplot(2,1,2)
hold on
plot(gyro_F(:,1)-t_T(1),rates.dataset(set).AI)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,6)*(180/pi))
title("Inversion Rates at Foot Sensor (Decoupled)")
ylabel("Degrees/s")
xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

%%
save('JointRates.mat','rates');
%% plot coupled 

figure()

subplot(5,2,1)
hold on
plot(t_T-t_T(1),b1)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,2)*(180/pi))
%title("Flexion Rates at Thigh Sensor")
%ylabel("Degrees/s")
%xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

subplot(5,2,3)
hold on
plot(t_T-t_T(1),b2)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,3)*(180/pi))
%title("Abduction Rates at Thigh Sensor")
%ylabel("Degrees/s")
%xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

subplot(5,2,5)
hold on
plot(t_T-t_T(1),c1)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,4)*(180/pi))
%title("Flexion Rates at Shin Sensor (Coupled)")
%ylabel("Degrees/s")
%xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off 

subplot(5,2,7)
hold on
plot(t_F-t_T(1),d1)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,5)*(180/pi))
%title("Flexion Rates at Foot Sensor (Coupled)")
%ylabel("Degrees/s")
%xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

subplot(5,2,9)
hold on
plot(t_F-t_T(1),d2)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,6)*(180/pi))
%title("Inversion Rates at Foot Sensor (Coupled)")
%ylabel("Degrees/s")
xlabel("Time [s]")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

%% plot decoupled

%figure()

subplot(5,2,2)
hold on
plot(t_T-t_T(1),b1)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,2)*(180/pi))
%title("Flexion Rates at Thigh Sensor")
%ylabel("Degrees/s")
%xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

subplot(5,2,4)
hold on
plot(t_T-t_T(1),b2)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,3)*(180/pi))
%title("Abduction Rates at Thigh Sensor")
%ylabel("Degrees/s")
%xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

subplot(5,2,6)
hold on
plot(gyro_S(:,1)-t_T(1),rate_KF)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,4)*(180/pi))
%title("Flexion Rates at Shin Sensor (Coupled)")
%ylabel("Degrees/s")
%xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off 

subplot(5,2,8)
hold on
plot(gyro_F(:,1)-t_T(1),rate_AF)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,5)*(180/pi))
%title("Flexion Rates at Foot Sensor (Coupled)")
%ylabel("Degrees/s")
%xlabel("Time")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

subplot(5,2,10)
hold on
plot(gyro_F(:,1)-t_T(1),rate_AI)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,6)*(180/pi))
%title("Inversion Rates at Foot Sensor (Coupled)")
%ylabel("Degrees/s")
xlabel("Time [s]")
%legend("Calculated","Truth")
axis(axes_lim)
grid on
grid minor
hold off

%% Find rotation matrix for Ankle

sample_offset = 0;%50;
Shin = [Axes.KF';accel_S_rem(1,2:4)];
Foot = [Axes.AF';SensData.datasegment(set).Right.Foot.Accel(1+sample_offset,2:4)];


H = Shin'*Foot;
[U,S,V] = svd(H);
d = det(V*U');
R_FS_init = V*[1,0,0;0,1,0;0,0,d]*U';

Rv = eye(3);

%gyro_S = GyroShinTemp(1+sample_offset:end,:);
%gyro_T = GyroThighTemp(1:end-sample_offset,:);
gyro_S = gyro_S_rem(1+sample_offset:end,:);%GyroShinTemp(1+sample_offset:end,:); %%%%%%%%%%
gyro_F = GyroFootTemp(1:end-sample_offset,:);
accel_S = accel_S_rem(1+sample_offset:end,:);%AccelShinTemp(1+sample_offset:end,:);%%%%%%%%%5
%accel_F = AccelFootTemp(1:end-sample_offset,:);

v_AF = Axes.AF;%J2_pca(:,1);
theta_AF_prev = 0;
t_prev = gyro_F(1,1);
Rv = eye(3);
angle = [];
rate_AF = [];
for k=2:length(gyro_F(:,1))
    t = gyro_F(k,1);
    gyro_F_rem(k,1:4) = [gyro_F(k,1),(gyro_F(k,2:4)'-Rv*R_FS_init*gyro_S(k,2:4)')'];
    %accel_F_rem(k,1:4) = [accel_F(k,1),(accel_F(k,2:4)'-Rv*R_FS_init*accel_S(k,2:4)')'];    
    rate_AF(k,1) = dot(Axes.AF,gyro_F_rem(k,2:4)');%dot(J2_pca(:,1),gyro_S_rem(k,2:4)');
    rate_AI(k,1) = dot(Axes.AI,gyro_F_rem(k,2:4)');%dot(J2_pca(:,1),gyro_S_rem(k,2:4)');
    theta_dot_AF = rate_AF(k);
    theta_AF = theta_AF_prev + theta_dot_AF*(t-t_prev);
    Rv = cosd(theta_AF)*eye(3) + (1-cosd(theta_AF))*(v_AF*v_AF')+sind(theta_AF)*[0,-v_AF(3),v_AF(2);v_AF(3),0,-v_AF(1);-v_AF(2),v_AF(1),0];
 
    angle = [angle; theta_AF];
    theta_AF_prev = theta_AF;
    t_prev = t;
end

figure()
subplot(2,1,1)
hold on
plot(t_F-t_T(1),d1)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,5)*(180/pi))
title("Flexion Rates at Foot Sensor (Coupled)")
ylabel("Degrees/s")
xlabel("Time")
legend("Calculated","Truth")
grid on
grid minor
hold off

subplot(2,1,2)
hold on
plot(gyro_F(:,1)-t_T(1),rate_AF)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,5)*(180/pi))
title("Flexion Rates at Foot Sensor (Decoupled)")
ylabel("Degrees/s")
xlabel("Time")
legend("Calculated","Truth")
%axis([0 15 -150 150])
grid on
grid minor
hold off 

figure()
subplot(2,1,1)
hold on
plot(t_F-t_T(1),d2)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,6)*(180/pi))
title("Inversion Rates at Foot Sensor (Coupled)")
ylabel("Degrees/s")
xlabel("Time")
legend("Calculated","Truth")
grid on
grid minor
hold off

subplot(2,1,2)
hold on
plot(gyro_F(:,1)-t_T(1),rate_AI)
plot(NAOJointRates(:,1)-t_T(1),NAOJointRates(:,6)*(180/pi))
title("Inversion Rates at Foot Sensor (Decoupled")
ylabel("Degrees/s")
xlabel("Time")
legend("Calculated","Truth")
grid on
grid minor
hold off
%%




%% Find rotation matrix for Knee

sample_offset = 0;%50;
Thigh = [Axes.HF';SensData.datasegment(set).Right.Thigh.Accel(1,2:4)];
Shin = [Axes.KF';SensData.datasegment(set).Right.Shin.Accel(1+sample_offset,2:4)];

H = Thigh'*Shin;
[U,S,V] = svd(H);
d = det(V*U');
R_ST_init = V*[1,0,0;0,1,0;0,0,d]*U';

%syms theta2_dot
% vx = J2_pca(1,1);
% vy = J2_pca(2,1);
% vz = J2_pca(3,1);
% v = [vx;vy;vz];
%R = cosd(theta2_dot)*eye(3) + (1-cosd(theta2_dot))*...
 %   [vx*vx,vx*vy,vx*vz;vx*vy,vy*vy,vy*vz;vx*vz,vy*vz,vz*vz]...
%    +sind(theta2_dot)*[0,-vz,vy;vz,0,-vx;-vy,vx,0];
%R_ST = R_ST_init*R;
%%
Rv = eye(3);
%R_ST_init = eye(3);
%gyro_S = GyroShinTemp(1+sample_offset:end,:);
%gyro_T = GyroThighTemp(1:end-sample_offset,:);
gyro_T = GyroThighTemp(1+sample_offset:end,:);
gyro_S = GyroShinTemp(1:end-sample_offset,:);

v_KF = Axes.KF;%J2_pca(:,1);
theta_KF_prev = 0;
t_prev = gyro_S(1,1);
Rv = eye(3);
angle = [];
rate_KF = [];
for k=2:length(gyro_S(:,1))
    t = gyro_S(k,1);
    rate_HF(k,1) = dot(Axes.HF,gyro_T(k,2:4)');%dot(J1_pca(:,1),gyro_T(k,2:4)');
    gyro_S_rem(k,1:4) = [gyro_S(k,1),(gyro_S(k,2:4)'-Rv*R_ST_init*gyro_T(k,2:4)')'];
    %gyro_S_rem(k,1:4) = [gyro_S(k,1),(gyro_S(k,2:4)'-rate_HF(k,1)*J1_pca(:,1))'];    
    rate_KF(k,1) = dot(Axes.KF,gyro_S_rem(k,2:4)');%dot(J2_pca(:,1),gyro_S_rem(k,2:4)');
    theta_dot_KF = rate_KF(k);
    theta_KF = theta_KF_prev + theta_dot_KF*(t-t_prev);
    Rv = cosd(theta_KF)*eye(3) + (1-cosd(theta_KF))*(v_KF*v_KF')+sind(theta_KF)*[0,-v_KF(3),v_KF(2);v_KF(3),0,-v_KF(1);-v_KF(2),v_KF(1),0];
 
    angle = [angle; theta_KF];
    theta_KF_prev = theta_KF;
    t_prev = t;
end

figure()
subplot(2,1,1)
hold on
plot(t_S-t_T(1),c1)
title("Flexion Rates at Shin Sensor (Coupled)")
ylabel("Degrees/s")
xlabel("Time")
%axis([0 15 -150 150])
grid on
grid minor
hold off 

subplot(2,1,2)
hold on
plot(gyro_S(:,1)-t_T(1),rate_KF)
title("Flexion Rates at Shin Sensor (Decoupled)")
ylabel("Degrees/s")
xlabel("Time")
%axis([0 15 -150 150])
grid on
grid minor
hold off 


%% 
offset = 0;%-.2;
%load('Truth_Time.mat')
%load('KF_Rate.mat')
%timestart = Truth_Time(1);
%Truth_Time_resamp = timestart:.01:Truth_Time(end);
%KF_Rate_resamp = interp1(Truth_Time, KF_Rate, Truth_Time_resamp,'pchip');

figure()
hold on
plot(gyro_S(:,1)-gyro_S(1,1),rate_KF)%-timestart+offset,rate_KF)
%plot(Truth_Time_resamp-timestart,-KF_Rate_resamp)
%plot(Truth_Time-timestart,-KF_Rate)
title('Knee Flexion Joint Rates')
%legend('Calculated','Truth')
xlabel('Time')
ylabel('Degrees/s')
%axis([0 115 -150 150])




%%
%%
rates = zeros(length(SensData.datasegment(1).Right.Thigh.Accel(:,1)),1);
%R_ST_current = R_ST_init;
x0 = 0;
t1 = [];
x1 = [];
angle = [];
v_HF = R_ST_init*J1_pca(:,1);
v_HA = R_ST_init*J1_pca(:,2);
v_KF = J2_pca(:,1);
theta_KF_prev = 0;
t_prev = SensData.datasegment(1).Right.Shin.Gyro(1+sample_offset,1);
Rv = eye(3);
gyro_S_rem = [];
for i=2:50:length(SensData.datasegment(1).Right.Shin.Gyro(:,1))-sample_offset
    t = SensData.datasegment(1).Right.Shin.Gyro(i+sample_offset,1);
    gyro_S = SensData.datasegment(1).Right.Shin.Gyro(i+sample_offset,2:4)';
    gyro_T = SensData.datasegment(1).Right.Thigh.Gyro(i,2:4)';
    
    R_ST = Rv*R_ST_init;
    temp = gyro_S - b1(i)*R_ST*v_HF - b2(i)*R_ST*v_HA;
    gyro_S_rem = [gyro_S_rem; temp];
    theta_dot_KF = dot(v_KF,temp);
    
    t1 = [t1,t];
    x1 = [x1,theta_dot_KF];
    %x0 = [x(1:9);0];
    
    theta_KF = theta_KF_prev + theta_dot_KF*(t-t_prev);
    Rv = cosd(theta_KF)*eye(3) + (1-cosd(theta_KF))*(v_KF*v_KF')+sind(theta_KF)*[0,-v_KF(3),v_KF(2);v_KF(3),0,-v_KF(1);-v_KF(2),v_KF(1),0];
    
    theta_KF_prev = theta_KF;
    t_prev = t;
    angle = [angle; theta_KF];
end
 
figure()
hold on
timestart = t1(1);
plot(t1-timestart-.4,-x1)
plot(t2-timestart,x2)
legend('Calculated','Truth')
xlabel('Time')
ylabel('Joint Rates')
axis([0 115 -150 150])

 %%
rates = zeros(length(SensData.datasegment(1).Right.Thigh.Accel(:,1)),1);
%R_ST_current = R_ST_init;
x0 = 0;
t1 = [];
x1 = [];
angle = [];
v_KF = J2_pca(:,1);
theta_KF_prev = 0;
t_prev = SensData.datasegment(1).Right.Thigh.Accel(1,1);
for i=2:50:1000%length(SensData.datasegment(1).Right.Thigh.Accel(:,1))
    t = SensData.datasegment(1).Right.Thigh.Accel(i,1);
    gyro_S = SensData.datasegment(1).Right.Shin.Gyro(i,2:4)';
    gyro_T = SensData.datasegment(1).Right.Thigh.Gyro(i,2:4)';
    x = fmincon(@(x) objfun5(x,v_KF,gyro_S,gyro_T,R_ST_init,theta_KF_prev,t,t_prev),x0,[],[],[],[],[],[]);
    theta_KF = theta_KF_prev + x*(t-t_prev);
    %x = fmincon(@(x)objfun(x,R_ST_current',v,gyro_S,gyro_T),x0,[],[],[],[],[],[],@(x)nlcon(x,v));
    %R_ST_current = reshape(x(1:9),[3,3]);
    rates(i) = x;
    t1 = [t1,SensData.datasegment(1).Right.Thigh.Accel(i,1)];
    x1 = [x1,rates(i)];
    %x0 = [x(1:9);0];
    
    theta_KF_prev = theta_KF;
    t_prev = t;
    angle = [angle; theta_KF];
end
 
figure()
hold on
timestart = t1(1);
plot(t1-timestart-.4,-x1)
plot(t2-timestart,x2)
legend('Calculated','Truth')
xlabel('Time')
ylabel('Joint Rates')
axis([0 115 -150 150])
 
 
 %%
rates = zeros(length(SensData.datasegment(1).Right.Thigh.Accel(:,1)),1);
%R_ST_current = R_ST_init;
x0 = [reshape(eye(3),[9,1]);0];
t1 = [];
x1 = [];
v_KF = J2_pca(:,1);
v_HF = R_ST_init*J1_pca(:,1);
v_HA = R_ST_init*J1_pca(:,2);
for i=1:25:length(SensData.datasegment(1).Right.Thigh.Accel(:,1))
    theta_dot_HF = b1(i);
    theta_dot_HA = b2(i);
    gyro_S = SensData.datasegment(1).Right.Shin.Gyro(i,2:4)';
    x = fmincon(@(x)objfun4(x,v_KF,gyro_S,theta_dot_HF,theta_dot_HA,v_HA),x0,[],[],[],[],[],[],@(x)nlcon4(x,v_KF,v_HF));
    %x = fmincon(@(x)objfun(x,R_ST_current',v,gyro_S,gyro_T),x0,[],[],[],[],[],[],@(x)nlcon(x,v));
    %R_ST_current = reshape(x(1:9),[3,3]);
    rates(i) = x(end);
    t1 = [t1,SensData.datasegment(1).Right.Thigh.Accel(i,1)];
    x1 = [x1,rates(i)];
    %x0 = [x(1:9);0];
end
 
figure()
hold on
timestart = t1(1);
plot(t1-timestart-.4,-x1)
%plot(t2-timestart,x2)
%legend('Calculated','Truth')
xlabel('Time')
ylabel('Joint Rates')
axis([0 115 -150 150])
 %%
 %%
rates = zeros(length(SensData.datasegment(1).Right.Thigh.Accel(:,1)),1);
R_ST_current = R_ST_init;
x0 = [reshape(eye(3),[9,1]);0];
t1 = [];
x1 = [];
for i=1:25:length(SensData.datasegment(1).Right.Thigh.Accel(:,1))-50
    x0 = [reshape(eye(3),[9,1]);0];
    gyro_T = SensData.datasegment(1).Right.Thigh.Gyro(i,2:4)';
    gyro_S = SensData.datasegment(1).Right.Shin.Gyro(i+sample_offset,2:4)';
    x = fmincon(@(x)objfun(x,R_ST_init',v,gyro_S,gyro_T),x0,[],[],[],[],[],[],@(x)nlcon(x,v));
    %x = fmincon(@(x)objfun(x,R_ST_current',v,gyro_S,gyro_T),x0,[],[],[],[],[],[],@(x)nlcon(x,v));
    %R_ST_current = reshape(x(1:9),[3,3]);
    rates(i) = x(end)+2*b1(i);
    t1 = [t1,SensData.datasegment(1).Right.Thigh.Accel(i,1)];
    x1 = [x1,rates(i)];
    %x0 = [x(1:9);0];
end
 
 %fun = @(theta2_dot) -SensData.datasegment(1).Right.Shin.Gyro(1,2:4)'+R_ST*SensData.datasegment(1).Right.Thigh.Gyro(1,2:4)'...
%    +theta2_dot*J2_pca(:,1)';
%solve(fun,theta2_dot)
%a = fminsearch(fun,0)
%%
rates = zeros(length(SensData.datasegment(1).Right.Thigh.Accel(:,1)),1);
R_ST_current = R_ST_init;
x0 = [reshape(eye(3),[9,1]);0];
t1 = [];
x1 = [];
for i=1:25:length(SensData.datasegment(1).Right.Thigh.Accel(:,1))
    x0 = [reshape(eye(3),[9,1]);0];
    gyro_T = SensData.datasegment(1).Right.Thigh.Gyro(i,2:4)'-b1(i)*J1_pca(:,1); %thigh gyro without the flexion part
    %gyro_T = b2(i)*J2_pca(:,1);
    gyro_S = SensData.datasegment(1).Right.Shin.Gyro(i,2:4)';
    x = fmincon(@(x)objfun3(x,R_ST_init,v,gyro_S,gyro_T,-b1(i)),x0,[],[],[],[],[],[],@(x)nlcon(x,v));
    %x = fmincon(@(x)objfun3(x,R_ST_current,v,gyro_S,gyro_T,-b1(i)),x0,[],[],[],[],[],[],@(x)nlcon(x,v));
   % R_ST_current = reshape(x(1:9),[3,3]);
    rates(i) = x(end);
    t1 = [t1,SensData.datasegment(1).Right.Thigh.Accel(i,1)];
    x1 = [x1,rates(i)];
    %x0 = [x(1:9);0];
end
 
 %fun = @(theta2_dot) -SensData.datasegment(1).Right.Shin.Gyro(1,2:4)'+R_ST*SensData.datasegment(1).Right.Thigh.Gyro(1,2:4)'...
%    +theta2_dot*J2_pca(:,1)';
%solve(fun,theta2_dot)
%a = fminsearch(fun,0)
%%
% x0 = 0;
% t1 = [];
% x1 = [];
% for i=1:length(SensData.datasegment(1).Right.Thigh.Accel(:,1))
%     %x0 = [reshape(eye(3),[9,1]);0];
%     gyro_S = SensData.datasegment(1).Right.Shin.Gyro(i,2:4)';
%     x = fmincon(@(x)objfun2(x,R_ST_init,v,b1(i), J1_pca(:,1),b2(i),J1_pca(:,2),gyro_S),x0,[],[]);
%     %R_ST_current = reshape(x(1:9),[3,3]);
%     rates(i) = x;
%     t1 = [t1,SensData.datasegment(1).Right.Thigh.Accel(i,1)];
%     x1 = [x1,rates(i)];
% end
%%
figure()
hold on
timestart = t1(1);
plot(t1-timestart-.4,-x1)
plot(t2-timestart,x2)
legend('Calculated','Truth')
xlabel('Time')
ylabel('Joint Rates')
axis([0 115 -150 150])
%%
figure()
hold on
timestart = t1(1);
plot(t1-timestart,-x1)
plot(SensData.datasegment(1).Right.Thigh.Accel(:,1)-SensData.datasegment(1).Right.Thigh.Accel(1,1),2*b1)
legend('Calculated KF Rate','Calculated HF Rate')
xlabel('Time')
ylabel('Joint Rates')
axis([0 115 -150 150])
%%
dx = x(2:end) - x(1:end-1);
dt = t(2:end) - t(1:end-1);
v = dx./dt;