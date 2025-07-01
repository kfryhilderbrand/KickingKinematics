%%
load('SensorData.mat')
load('HF_Ang_deg.mat')
close all
startt = 1;%5000;
endt = startt + 10000;
GyroThighTemp = SensData.datasegment(1).Right.Thigh.Gyro(startt:endt,:);
GyroShinTemp = SensData.datasegment(1).Right.Shin.Gyro;
t_T = SensData.datasegment(1).Right.Thigh.Gyro(startt:endt,1);
t_S = SensData.datasegment(1).Right.Shin.Gyro(:,1);
h=GyroThighTemp(2,1)-GyroThighTemp(1,1);
%% Looking at y
b=find([abs(diff(GyroThighTemp(:,3)))' inf]>1)';
lngths=diff([0 b'])'; %length of the sequences
ends=(cumsum(lngths'))'; %endpoints of the sequences
seqind = find(lngths>=100);
edgs = [ends(seqind)-lngths(seqind)+1,ends(seqind)];
%modes = [];
means = [];
GyroThighTemp_Adj=GyroThighTemp;
for i=1:length(seqind)
  
        %modes = [modes; mode(GyroThighTemp_Adj(edgs(i,1):edgs(i,2),3))];
        means = [means; mean(GyroThighTemp_Adj(edgs(i,1):edgs(i,2),3))];
        GyroThighTemp_Adj(edgs(i,1):end,3) = GyroThighTemp_Adj(edgs(i,1):end,3) - means(i);%- modes(i);
end
%GyroThighTemp = GyroThighTemp_Adj;
%[n, ~] = size(GyroThighTemp);
%%
anglesx = cumtrapz(t_T-t_T(1),GyroThighTemp(:,2));
anglesy = cumtrapz(t_T-t_T(1),GyroThighTemp(:,3));
anglesz = cumtrapz(t_T-t_T(1),GyroThighTemp(:,4));

figure()
plot(t_T-t_T(1),anglesx)
title('xangles')
figure()
%plot(t_T-t_T(1),anglesy)
%plot(GyroThighTemp_Adj(:,3))
plot(anglesy)
title('yangles')
figure()
plot(t_T-t_T(1),anglesz)
title('zangles')
figure()
plot(HF_Ang)
title('HF')

%%
%%%double differentiation
angleratex = [anglesx(1);diff(anglesx)/h];
angleratey = [anglesy(1);diff(anglesy)/h]; %GyroThighTemp_Adj(:,3)
angleratez = [anglesz(1);diff(anglesz)/h];

angleaccelx = [angleratex(1)/h;diff(angleratex)/h];
angleaccely = [angleratey(1)/h;diff(angleratey)/h];
%  for i = 1:size(edgs,1)-1
%      angleaccely(edgs(i,1):edgs(i,2)-1) = [angleratey(edgs(i,1));diff(angleratey(edgs(i,1):edgs(i,2)-1))/h];
%      angleaccely(edgs(i,2):edgs(i+1,1)) = diff(angleratey(edgs(i,2)-1:edgs(i+1,1)))/h;
%  end
angleaccelz = [angleratez(1)/h;diff(angleratez)/h];

%%double integration
init_rate = [0,0,0];

angleratexint = cumtrapz(t_T-t_T(1),angleaccelx)+init_rate(1);
%anglerateyint = cumtrapz(t_T-t_T(1),angleaccely)+init_rate(2);
offset = 0; 
for i = 3:3%size(edgs,1)-1
    % anglerateyint(edgs(i,1):edgs(i,2)) = cumtrapz(t_T(edgs(i,1):edgs(i,2))-t_T(1),angleaccely(edgs(i,1):edgs(i,2)))+init_rate(2) - offset;
     anglerateyint(edgs(i,2)+1:edgs(i+1,1)) = cumtrapz(t_T(edgs(i,2)+1:edgs(i+1,1))-t_T(1),angleaccely(edgs(i,2)+1:edgs(i+1,1)))+init_rate(2);
     %anglerateyint(edgs(i,1):edgs(i,2)) = anglerateyint(edgs(i,1):edgs(i,2))-anglerateyint(edgs(i,2));
     offset = 0;%anglerateyint(edgs(i+1,1));
end
%anglerateyint(edgs(i,1):edgs(i,2)) = cumtrapz(t_T(edgs(i,1):edgs(i,2))-t_T(1),angleaccely(edgs(i,1):edgs(i,2)))+init_rate(2) - offset;
angleratezint = cumtrapz(t_T-t_T(1),angleaccelz)+init_rate(3);
%%
anglesxint = cumtrapz(t_T-t_T(1),angleratexint);
% anglesyint = cumtrapz(t_T-t_T(1),anglerateyint);
%offset = 0;
 for i = 3:3%1:size(edgs,1)-1
      anglesyint(edgs(i,1):edgs(i,2)-1) = cumtrapz(t_T(edgs(i,1):edgs(i,2)-1)-t_T(1),anglerateyint(edgs(i,1):edgs(i,2)-1)) - offset;
      anglesyint(edgs(i,2):edgs(i+1,1)) = cumtrapz(t_T(edgs(i,2):edgs(i+1,1))-t_T(1),anglerateyint(edgs(i,2):edgs(i+1,1)));
 end
%anglesyint(edgs(i,1):edgs(i,2)-1) = cumtrapz(t_T(edgs(i,1):edgs(i,2)-1)-t_T(1),anglerateyint(edgs(i,1):edgs(i,2)-1)) - offset;
angleszint = cumtrapz(t_T-t_T(1),angleratezint);
%%
figure()
plot(t_T-t_T(1),anglesxint)
title('xangles')
figure()
plot(t_T-t_T(1),anglesyint)
title('yangles')
figure()
plot(t_T-t_T(1),angleszint)
title('zangles')

%%
figure()
hold on
plot(t_T-t_T(1),GyroThighTemp(:,4),'-r')
plot(t_T-t_T(1),angleratez,'-b')
plot(t_T-t_T(1),angleratezint,'-g')
% plot(t_S-t_T(1),anglesz,'-r')
% plot(t_S-t_T(1),anglezint,'-b')

close all
load('KF_Rate.mat')
KF_Angrate = [KF_Rate(1);diff(KF_Rate)];
KF_Rate_Adj = KF_Rate;
flip_Flag = 1;
for i = 1:length(KF_Rate)
    if abs(KF_Angrate(i))<=0.5 
       % if flip_Flag == 1
            KF_Rate_Adj(i:end) = KF_Rate_Adj(i:end)-KF_Rate_Adj(i);
        %    flip_Flag = 0;
      %  end
    %else
    %    flip_Flag = 1;
    end
end
figure()
hold on
plot(KF_Rate_Adj)
plot(KF_Rate)
legend('KF_Rate','KF_Rate_Adj')

b=find([abs(KF_Angrate') inf]>1)';
lngths=diff([0 b'])'; %length of the sequences
ends=(cumsum(lngths'))'; %endpoints of the sequences
seqind = find(lngths>=3);
edgs = [ends(seqind)-lngths(seqind)+1,ends(seqind)];
modes = [];
for i=1:length(seqind)-1
  
        modes = [modes; mode(KF_Rate(edgs(i,1):edgs(i,2)))];
    
end


%%
figure()
hold on
plot(cumtrapz(KF_Rate))
plot(cumtrapz(KF_Rate_Adj))
legend('KF_Ang','KF_Ang_Adj')
