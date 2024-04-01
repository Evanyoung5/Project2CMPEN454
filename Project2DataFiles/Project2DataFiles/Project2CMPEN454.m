

[CamOneCoord, CamTwoCoord] = ThreeDtoTwoDConversion('Parameters_V1.mat','Parameters_V2.mat');

% Gathering parameters for cameras one and two
ParamOneObject = matfile('Parameters_V1.mat');
ParamTwoObject = matfile('Parameters_V2.mat');

positionOne = getfield(ParamOneObject.Parameters(1,1),'position');
rotationOne = getfield(ParamOneObject.Parameters(1,1),'Rmat');
KmatrixOne = getfield(ParamOneObject.Parameters(1,1),'Kmat');

positionTwo = getfield(ParamTwoObject.Parameters(1,1),'position');
rotationTwo = getfield(ParamTwoObject.Parameters(1,1),'Rmat');
KmatrixTwo = getfield(ParamTwoObject.Parameters(1,1),'Kmat');

positionOne = transpose(positionOne);
positionTwo = transpose(positionTwo);

COne = -transpose(rotationOne)*positionOne;
CTwo = -transpose(rotationTwo)*positionTwo;

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