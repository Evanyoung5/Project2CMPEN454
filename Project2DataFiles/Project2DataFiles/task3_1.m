function [CamOneCoord, CamTwoCoord] = task3_1(paramOne,paramTwo)

%%% Working 3.1 Function

ParamOneObject = matfile(paramOne);
ParamTwoObject = matfile(paramTwo);

%Camera One Calculations
positionOne = getfield(ParamOneObject.Parameters(1,1),'position');
rotationOne = getfield(ParamOneObject.Parameters(1,1),'Rmat');
KmatrixOne = getfield(ParamOneObject.Parameters(1,1),'Kmat');
SMatrixOne = [1 0 0 -positionOne(1);0 1 0 -positionOne(2); 0 0 1 -positionOne(3); 0 0 0 1];
foclengthOne = getfield(ParamOneObject.Parameters(1,1),'foclen');
RMatrixOne = [rotationOne(1,1) rotationOne(1,2) rotationOne(1,3) 0;rotationOne(2,1) rotationOne(2,2) rotationOne(2,3) 0;rotationOne(3,1) rotationOne(3,2) rotationOne(3,3) 0;0 0 0 1];




ExtendedKmatOne = KmatrixOne*[1 0 0 0; 0 1 0 0; 0 0 1 0];
WorldtoCamMatrixOne = ExtendedKmatOne * RMatrixOne * SMatrixOne;

%Camera Two Calculations
positionTwo = getfield(ParamTwoObject.Parameters(1,1),'position');
rotationTwo = getfield(ParamTwoObject.Parameters(1,1),'Rmat');
KmatrixTwo = getfield(ParamTwoObject.Parameters(1,1),'Kmat');
SMatrixTwo = [1 0 0 -positionTwo(1);0 1 0 -positionTwo(2); 0 0 1 -positionTwo(3); 0 0 0 1];
foclengthTwo = getfield(ParamTwoObject.Parameters(1,1),'foclen');
RMatrixTwo = [rotationTwo(1,1) rotationTwo(1,2) rotationTwo(1,3) 0;rotationTwo(2,1) rotationTwo(2,2) rotationTwo(2,3) 0;rotationTwo(3,1) rotationTwo(3,2) rotationTwo(3,3) 0;0 0 0 1];




ExtendedKmatTwo = KmatrixTwo*[1 0 0 0; 0 1 0 0; 0 0 1 0];
WorldtoCamMatrixTwo = ExtendedKmatTwo * RMatrixTwo * SMatrixTwo;


%Reading in mocap data
mocapData = matfile('mocapPoints3D.mat');
mocapPosition = mocapData.pts3D;


[M,N] = size(mocapPosition);

CamOneCoord = zeros(2,N);
CamTwoCoord = zeros(2,N);

%Converting to pixel Coords
for i=1:N 
    U = mocapPosition(1,i);
    V = mocapPosition(2,i);
    W = mocapPosition(3,i);
    WorldCoord = [U; V; W; 1];
    pixCoordOne = WorldtoCamMatrixOne * WorldCoord;
    CamOneCoord(1,i) = pixCoordOne(1)/pixCoordOne(3);
    CamOneCoord(2,i) = pixCoordOne(2)/pixCoordOne(3);

    pixCoordTwo = WorldtoCamMatrixTwo * WorldCoord;
    CamTwoCoord(1,i) = pixCoordTwo(1)/pixCoordTwo(3);
    CamTwoCoord(2,i) = pixCoordTwo(2)/pixCoordTwo(3);

end

return
imOne = imread("im1corrected.jpg");
imTwo = imread("im2corrected.jpg");
impixel(imTwo)
imshow(imOne);
axis on;
hold on;

%Plots locations of mocap data onto image 1
plot(CamOneCoord(1,:),CamOneCoord(2,:),'r+','MarkerSize',10);