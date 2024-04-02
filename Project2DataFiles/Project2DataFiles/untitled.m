%Calculating MSE
summation = 0;
for j=1:N
    dist = sum((NewWorldCoords(:,j)-mocapPosition(:,j)).^2);
    summation=summation + dist;


end
MSE = summation/N;

%imshow(imOne);
%axis on;
%hold on;

%Plots locations of mocap data onto image 1
%plot(CamOneCoord(1,:),CamOneCoord(2,:),'r+','MarkerSize',10);

VectorOne = NewWorldCoords(:,2)-NewWorldCoords(:,1);
VectorTwo = NewWorldCoords(:,3)-NewWorldCoords(:,1);

NVector = cross(VectorOne,VectorTwo);