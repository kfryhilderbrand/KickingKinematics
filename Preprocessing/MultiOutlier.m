%d2 = mahal(Y,X)
d=[];
[m, ~] = size(X);
for i = 1:m
    Y = X;
    Y(i,:) = [];
    d2 = mahal(X(i,:),Y);
    d = [d; d2];
end

%%

[m, ~] = size(X);
numsamps = round(.6*m);
reps = 15;
d = zeros(m,5);
for j = 1:reps
   indx = [];
   Y = [];
   
   indx = randperm(m,numsamps); %randi([1 m],numsamps, 1);
   Y = X(indx,:);
   for i=1:m
       if ~ismember(i,indx)
           d2 = mahal(X(i,:),Y)';
           d(i,j) = d2;
       end
   end
end