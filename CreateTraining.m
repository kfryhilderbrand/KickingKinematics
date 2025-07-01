%% Function to create and gather training data
%%%Training data created using a leave-one-out approach. Each set of 
%%%training data contains SHOD features for individual sensors, window 
%%%Accel and Gyro data for individual sensors and overall SHOD data
%% Inputs:
%%%Data:         Struct containing sensor data for windows
%%%Act:          Struct containing SHOD features for windows
%%%Truth:        Struct containing truth data for windows

%% Outputs:
%%%Training:    Struct containing training data for each data segment.
            

function [Training]=CreateTraining(Data, Act, Truth)

%%%Initialize Training struct
for i=1:length(Act.datasegment)
    f = [fieldnames(Data.datasegment(i).Left.Foot); fieldnames(Act.datasegment(i).Left.Foot)];
    for k = 1:length(f) %%%Loop through all fields of Data structure
        Training.datasegment(i).Left.Foot.(f{k}) = [];
        Training.datasegment(i).Left.Shin.(f{k}) = [];
        Training.datasegment(i).Left.Thigh.(f{k}) = [];
        
        Training.datasegment(i).Right.Foot.(f{k}) = [];
        Training.datasegment(i).Right.Shin.(f{k}) = [];
        Training.datasegment(i).Right.Thigh.(f{k}) = [];        
    end
    
    Training.datasegment(i).Left.SHOD3D = [];
    Training.datasegment(i).Left.WinTime = [];
    Training.datasegment(i).Left.WinTruth = [];
    
    Training.datasegment(i).Right.SHOD3D = [];
    Training.datasegment(i).Right.WinTime = [];
    Training.datasegment(i).Right.WinTruth = [];
end

%%%Loop through each data segment to create training for
for i=1:length(Act.datasegment)
    %%%Loop through data segments and concatenate to training data if i!=j
    for j=1:length(Act.datasegment)
        if i ~= j
            %%% Concatenate individual sensor data and window times into training data
            f = fieldnames(Data.datasegment(i).Left.Foot);
            for k = 1:length(f) %%%Loop through all fields of Data structure
                %%%Foot
                Training.datasegment(i).Left.Foot.(f{k}) = [Training.datasegment(i).Left.Foot.(f{k}); ...
                    Data.datasegment(j).Left.Foot.(f{k})];
                Training.datasegment(i).Right.Foot.(f{k}) = [Training.datasegment(i).Right.Foot.(f{k}); ...
                    Data.datasegment(j).Right.Foot.(f{k})];
                %%%Shin
                Training.datasegment(i).Left.Shin.(f{k}) = [Training.datasegment(i).Left.Shin.(f{k}); ...
                    Data.datasegment(j).Left.Shin.(f{k})];
                Training.datasegment(i).Right.Shin.(f{k}) = [Training.datasegment(i).Right.Shin.(f{k}); ...
                    Data.datasegment(j).Right.Shin.(f{k})];
                %%%Thigh
                Training.datasegment(i).Left.Thigh.(f{k}) = [Training.datasegment(i).Left.Thigh.(f{k}); ...
                    Data.datasegment(j).Left.Thigh.(f{k})];
                Training.datasegment(i).Right.Thigh.(f{k}) = [Training.datasegment(i).Right.Thigh.(f{k}); ...
                    Data.datasegment(j).Right.Thigh.(f{k})];
            end
            
            %%%Concatenate individual SHOD features into training data
            f = fieldnames(Act.datasegment(i).Left.Foot);
            for k = 1:length(f) %%%Loop through all fields of Data structure
                %%%Foot                
                Training.datasegment(i).Left.Foot.(f{k}) = [Training.datasegment(i).Left.Foot.(f{k}); ...
                    Act.datasegment(j).Left.Foot.(f{k})];
                Training.datasegment(i).Right.Foot.(f{k}) = [Training.datasegment(i).Right.Foot.(f{k}); ...
                    Act.datasegment(j).Right.Foot.(f{k})];
                %%%Shin              
                Training.datasegment(i).Left.Shin.(f{k}) = [Training.datasegment(i).Left.Shin.(f{k}); ...
                    Act.datasegment(j).Left.Shin.(f{k})];
                Training.datasegment(i).Right.Shin.(f{k}) = [Training.datasegment(i).Right.Shin.(f{k}); ...
                    Act.datasegment(j).Right.Shin.(f{k})];
                %%%Thigh             
                Training.datasegment(i).Left.Thigh.(f{k}) = [Training.datasegment(i).Left.Thigh.(f{k}); ...
                    Act.datasegment(j).Left.Thigh.(f{k})];
                Training.datasegment(i).Right.Thigh.(f{k}) = [Training.datasegment(i).Right.Thigh.(f{k}); ...
                    Act.datasegment(j).Right.Thigh.(f{k})];                
            end
            
            %%%Concatenate overall SHOD features and window time into training data
            Training.datasegment(i).Left.SHOD3D = [Training.datasegment(i).Left.SHOD3D; ...
                Act.datasegment(j).Left.SHOD3D];
            Training.datasegment(i).Right.SHOD3D = [Training.datasegment(i).Right.SHOD3D; ...
                Act.datasegment(j).Right.SHOD3D];
            Training.datasegment(i).Left.WinTime = [Training.datasegment(i).Left.WinTime; ...
                Act.datasegment(j).Left.WinTime];
            Training.datasegment(i).Right.WinTime = [Training.datasegment(i).Right.WinTime; ...
                Act.datasegment(j).Right.WinTime]; 
            
            %%%Concatenate truth data for windows into training data
            Training.datasegment(i).Left.WinTruth = [Training.datasegment(i).Left.WinTruth; ...
                Truth.datasegment(j).Left.WinTruth];
            Training.datasegment(i).Right.WinTruth = [Training.datasegment(i).Right.WinTruth; ...
                Truth.datasegment(j).Right.WinTruth];
            
        end
    end   
end

end