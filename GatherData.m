%% Function to reshape .mat file in current directory.  
%This function will organize data into a cell array X that will combine 
%individual data snippets (like individual sensors) in the .mat file.
%The number of cells for X will be determined by the number of sensors and
%the number of cells for each cell of X will be determined by NumMeas.

%% Inputs: 
%Data:
%NumMeas: number of measurements per sensor (like 2 for accel and gyro)
%HFSflag: flag indicating wether high frequency streaming was used, if so 
%         EPOCH time within each packet is adjusted)
%fs:      sampling frequency in Hz
%% Outputs:
%X:       Reshaped Data cell array
%% NOTE: 
%Different measurements for each sensor (Accel and Gyro) are currently 
%written to separate files

function [X]=GatherData(Data, NumMeas, HFSflag, fs)

[~,len] = size(Data);  %len is number of individual sensors * NumMeas(i.e. number of cells in data)
X = cell(1,len/NumMeas);

k=1;
for i = 1:NumMeas:len
    X{k} = cell(1,NumMeas);
    for j=1:NumMeas
        X{k}{j} = Data{i+j-1}{1};%(:,2:end);  %start at 2 b/c 1 is the time (not EPOCH)
    end
    k=k+1;
end


%If HFS used, separates packets (adjust second and third member of each packet)
if HFSflag
    for i=1:length(X)
        for j=1:length(X{i})
            for k=1:3:length(X{i}{j}(:,1))
                if length(X{i}{j}(:,1))-k==0   %to deal with incomplete end packets
                     X{i}{j}(k,1)=X{i}{j}(k,1);
                elseif length(X{i}{j}(:,1))-k==1
                    X{i}{j}(k,1)=X{i}{j}(k,1);
                    X{i}{j}(k+1,1)=X{i}{j}(k,1)+1/fs;
                elseif length(X{i}{j}(:,1))-k==2
                    X{i}{j}(k,1)=X{i}{j}(k,1);
                    X{i}{j}(k+1,1)=X{i}{j}(k,1)+1/fs;
                    X{i}{j}(k+2,1)=X{i}{j}(k,1)+2/fs;
                else
                    X{i}{j}(k,1)=X{i}{j}(k,1);
                    X{i}{j}(k+1,1)=X{i}{j}(k,1)+(X{i}{j}(k+3,1)-X{i}{j}(k,1))/3;
                    X{i}{j}(k+2,1)=X{i}{j}(k,1)+2*(X{i}{j}(k+3,1)-X{i}{j}(k,1))/3;
                end
            end
        [~,idx]=unique(X{i}{j}(:,1),'stable');%don't sort
        temp = X{i}{j}(idx,:);
        X{i}{j}=temp;
        end
    end
end