

[CamOneCoord, CamTwoCoord] = task3_1('Parameters_V1.mat','Parameters_V2.mat');

% Gathering parameters for cameras one and two
ParamOneObject = matfile('Parameters_V1.mat');
ParamTwoObject = matfile('Parameters_V2.mat');

positionOne = getfield(ParamOneObject.Parameters(1,1),'position');
rotationOne = getfield(ParamOneObject.Parameters(1,1),'Rmat');
KmatrixOne = getfield(ParamOneObject.Parameters(1,1),'Kmat');
SMatrixOne = [1 0 0 -positionOne(1);0 1 0 -positionOne(2); 0 0 1 -positionOne(3); 0 0 0 1];
RMatrixOne = [rotationOne(1,1) rotationOne(1,2) rotationOne(1,3) 0;rotationOne(2,1) rotationOne(2,2) rotationOne(2,3) 0;rotationOne(3,1) rotationOne(3,2) rotationOne(3,3) 0;0 0 0 1];
JoinedMatrixOne = RMatrixOne * SMatrixOne;
TVectorOne = JoinedMatrixOne(1:3,4);


positionTwo = getfield(ParamTwoObject.Parameters(1,1),'position');
rotationTwo = getfield(ParamTwoObject.Parameters(1,1),'Rmat');
KmatrixTwo = getfield(ParamTwoObject.Parameters(1,1),'Kmat');
SMatrixTwo = [1 0 0 -positionTwo(1);0 1 0 -positionTwo(2); 0 0 1 -positionTwo(3); 0 0 0 1];
RMatrixTwo = [rotationTwo(1,1) rotationTwo(1,2) rotationTwo(1,3) 0;rotationTwo(2,1) rotationTwo(2,2) rotationTwo(2,3) 0;rotationTwo(3,1) rotationTwo(3,2) rotationTwo(3,3) 0;0 0 0 1];
JoinedMatrixTwo = RMatrixTwo * SMatrixTwo;
TVectorTwo = JoinedMatrixTwo(1:3,4);


COne = -transpose(rotationOne)*TVectorOne;
CTwo = -transpose(rotationTwo)*TVectorTwo;

%Reading in mocap data
mocapData = matfile('mocapPoints3D.mat');
mocapPosition = mocapData.pts3D;


[M,N] = size(mocapPosition);

CamOneVectors = zeros(2,N,3);
CamTwoVectors = zeros(2,N,3);

NewWorldCoords = zeros(3,N);
for i=1:N
    %Declares V1 and V2 and then converts them both to unit vectors
    VOne = transpose(rotationOne)*(KmatrixOne^(-1))*[CamOneCoord(1,i); CamOneCoord(2,i); 1];
    VTwo = transpose(rotationTwo)*(KmatrixTwo^(-1))*[CamTwoCoord(1,i); CamTwoCoord(2,i); 1];
    
    VOneMag = sqrt(sum(VOne'.^2));
    VOneUnit = VOne/VOneMag;

    VTwoMag = sqrt(sum(VTwo'.^2));
    VTwoUnit = VTwo/VTwoMag;
    
    %Assigns camera center
    CamOneVectors(1,i,:) = COne;
    CamOneVectors(2,i,:) = VOneUnit;

    CamTwoVectors(1,i,:) = CTwo;
    CamTwoVectors(2,i,:) = VTwoUnit;

    SolutionVal = CTwo - COne;
    %Creates u3 unit vector
    CrossVector = cross(VOneUnit,VTwoUnit);
    CrossUnitVector = CrossVector/(sqrt(sum(CrossVector'.^2)));
    SolutionMatrix = [VOneUnit CrossUnitVector -VTwoUnit];
    
    %Solves linear system of equations
    coefficients = linsolve(SolutionMatrix,SolutionVal);
    POne = COne + (coefficients(1)*VOneUnit);
    PTwo = CTwo + (coefficients(3)*VTwoUnit);
    
    %Finds new estimated point
    P= (POne + PTwo)/2;

    NewWorldCoords(1,i) = P(1);
    NewWorldCoords(2,i) = P(2);
    NewWorldCoords(3,i) = P(3);
end

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