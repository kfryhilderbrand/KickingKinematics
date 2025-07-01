%% Frequencies
close all
clear Rp Pp Rs Ps rp pp rs ps
[~,n] = size(Freq);
Data = [Ages, Freq];
Data = sortrows(Data,1);

[Rp,Pp] = corr(Data,'Type','Pearson');
rp = Rp(1,2:end);
pp = Pp(1,2:end);

[Rs,Ps] = corr(Data,'Type','Spearman');
rs = Rs(1,2:end);
ps = Ps(1,2:end);

MedData = [Ages_Med, Freq_Med];

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,1);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 1]);
    grid on
%     
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     rsq(i) = 1-SSres/SStot;                            % R^2
end

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,2);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 1]);
    grid on
%     
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     rsq2(i) = 1-SSres/SStot;                            % R^2
end

%% Durations
close all
clear Rp Pp Rs Ps rp pp rs ps
[~,n] = size(Dur);
Data = [Ages, Dur];
Data = sortrows(Data,1);

[Rp,Pp] = corr(Data,'Type','Pearson');
rp = Rp(1,2:end);
pp = Pp(1,2:end);

[Rs,Ps] = corr(Data,'Type','Spearman');
rs = Rs(1,2:end);
ps = Ps(1,2:end);

MedData = [Ages_Med, Dur_Med];

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,1);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 10]);
    grid on
%     
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq(i) = 1-SSres/SStot;                            % R^2
end

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    %plot(x,y,'xr')
    linCoeff = polyfit(x,y,2);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 10]);
    grid on
%     
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq2(i) = 1-SSres/SStot;                            % R^2
end

%% Accel
close all
clear Rp Pp Rs Ps rp pp rs ps
[~,n] = size(Accel);
Data = [Ages, Accel];
Data = sortrows(Data,1);

[Rp,Pp] = corr(Data,'Type','Pearson');
rp = Rp(1,2:end);
pp = Pp(1,2:end);

[Rs,Ps] = corr(Data,'Type','Spearman');
rs = Rs(1,2:end);
ps = Ps(1,2:end);

MedData = [Ages_Med, Accel_Med];
ind = [1,4,7,10];
for i=1:length(ind)%1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,ind(i)+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,ind(i)+1);%Freq(:,i);
    linCoeff = polyfit(x,y,1);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 5]);
    grid on
%     
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq(i) = 1-SSres/SStot;                            % R^2
end
%%
close all
for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,2);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 10]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq2(i) = 1-SSres/SStot;                            % R^2
end

%% AvgJointExc
close all
clear Rp Pp Rs Ps rp pp rs ps
[~,n] = size(AvgJointExc);
Data = [Ages, AvgJointExc];
Data = sortrows(Data,1);

[Rp,Pp] = corr(Data,'Type','Pearson');
rp = Rp(1,2:end);
pp = Pp(1,2:end);

[Rs,Ps] = corr(Data,'Type','Spearman');
rs = Rs(1,2:end);
ps = Ps(1,2:end);

MedData = [Ages_Med, AvgJointExc_Med];

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,1);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    if(mod(i,2)==1)
        axis([0 40 0 30]);
    else
        axis([0 40 -30 0]);
    end
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq(i) = 1-SSres/SStot;                            % R^2
end

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,2);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    if(mod(i,2)==1)
        axis([0 40 0 30]);
    else
        axis([0 40 -30 0]);
    end
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq2(i) = 1-SSres/SStot;                            % R^2
end

%% MaxJointExc
close all
clear Rp Pp Rs Ps rp pp rs ps
[~,n] = size(MaxJointExc);
Data = [Ages, MaxJointExc];
Data = sortrows(Data,1);

[Rp,Pp] = corr(Data,'Type','Pearson');
rp = Rp(1,2:end);
pp = Pp(1,2:end);

[Rs,Ps] = corr(Data,'Type','Spearman');
rs = Rs(1,2:end);
ps = Ps(1,2:end);

MedData = [Ages_Med, MaxJointExc_Med];

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,1);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 10]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq(i) = 1-SSres/SStot;                            % R^2
end

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,2);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 10]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq2(i) = 1-SSres/SStot;                            % R^2
end

%% MaxJointRates
close all
clear Rp Pp Rs Ps rp pp rs ps
[~,n] = size(MaxJointRates);
Data = [Ages, MaxJointRates];
Data = sortrows(Data,1);

[Rp,Pp] = corr(Data,'Type','Pearson');
rp = Rp(1,2:end);
pp = Pp(1,2:end);

[Rs,Ps] = corr(Data,'Type','Spearman');
rs = Rs(1,2:end);
ps = Ps(1,2:end);

MedData = [Ages_Med, MaxJointRates_Med];

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,1);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 10]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq(i) = 1-SSres/SStot;                            % R^2
end

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,2);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 10]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq2(i) = 1-SSres/SStot;                            % R^2
end

%% KickAmp
close all
clear Rp Pp Rs Ps rp pp rs ps
%[~,n] = size(KickAmp);
Data = [Ages, KickAmp];
Data = sortrows(Data,1);
Data=Data(sum(isnan(Data),2)==0,:);
[~,n] = size(Data);
n=n-1;

[Rp,Pp] = corr(Data,'Type','Pearson');
rp = Rp(1,2:end);
pp = Pp(1,2:end);

[Rs,Ps] = corr(Data,'Type','Spearman');
rs = Rs(1,2:end);
ps = Ps(1,2:end);

MedData = [Ages_Med, KickAmp_Med];

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,1);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 10]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq(i) = 1-SSres/SStot;                            % R^2
end

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,2);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 0 10]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq2(i) = 1-SSres/SStot;                            % R^2
end

%% InterJointCoor
close all
clear Rp Pp Rs Ps rp pp rs ps
%[~,n] = size(InterJointCoor);
Data = [Ages, InterJointCoor];
Data = sortrows(Data,1);
Data=Data(sum(isnan(Data),2)==0,:);
[~,n] = size(Data);
n=n-1;

[Rp,Pp] = corr(Data,'Type','Pearson');
rp = Rp(1,2:end);
pp = Pp(1,2:end);

[Rs,Ps] = corr(Data,'Type','Spearman');
rs = Rs(1,2:end);
ps = Ps(1,2:end);

MedData = [Ages_Med, InterJointCoor_Med];

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,1);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 -1 1]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq(i) = 1-SSres/SStot;                            % R^2
end

for i=1:n
    figure()
    hold on
    plot(MedData(:,1),MedData(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,2);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 -1 1]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq2(i) = 1-SSres/SStot;                            % R^2
end

%% Pred_MaxJointExc
close all
clear Rp Pp Rs Ps rp pp rs ps
[~,n] = size(Pred_MaxJointExc);
Data = [Ages, Pred_MaxJointExc];
Data = sortrows(Data,1);

[Rp,Pp] = corr(Data,'Type','Pearson');
rp = Rp(1,2:end);
pp = Pp(1,2:end);

[Rs,Ps] = corr(Data,'Type','Spearman');
rs = Rs(1,2:end);
ps = Ps(1,2:end);

for i=1:n
    figure()
    hold on
    plot(Data(:,1),Data(:,i+1),'xr')
    x = Data(:,1);%Ages;
    y = Data(:,i+1);%Freq(:,i);
    linCoeff = polyfit(x,y,1);
    yfit = polyval(linCoeff,x);
    plot(x,yfit,'--b')
    axis([0 40 -100 100]);
    grid on
    
%     SStot = sum((y-mean(y)).^2);                    % Total Sum-Of-Squares
%     SSres = sum((y-yfit).^2);                       % Residual Sum-Of-Squares
%     Rsq(i) = 1-SSres/SStot;                            % R^2
end
