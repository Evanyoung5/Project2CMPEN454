function [fundMat] = Fundamental(paramOne,paramTwo)

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

%Camera Two Calculations
positionTwo = getfield(ParamTwoObject.Parameters(1,1),'position');
rotationTwo = getfield(ParamTwoObject.Parameters(1,1),'Rmat');
KmatrixTwo = getfield(ParamTwoObject.Parameters(1,1),'Kmat');
SMatrixTwo = [1 0 0 -positionTwo(1);0 1 0 -positionTwo(2); 0 0 1 -positionTwo(3); 0 0 0 1];
foclengthTwo = getfield(ParamTwoObject.Parameters(1,1),'foclen');
RMatrixTwo = [rotationTwo(1,1) rotationTwo(1,2) rotationTwo(1,3) 0;rotationTwo(2,1) rotationTwo(2,2) rotationTwo(2,3) 0;rotationTwo(3,1) rotationTwo(3,2) rotationTwo(3,3) 0;0 0 0 1];

hPositionTwo=[positionTwo 1]';
%overall, camera one is left
newRot=rotationOne'*rotationTwo;%Rotational Matrix Calc
translat=RMatrixOne*SMatrixOne*hPositionTwo;%c2 position in c1 coords
tx=translat(1);
ty=translat(2);
tz=translat(3);
newSkew=[0 -tz ty;tz 0 -tx;-ty tx 0];%overall skew matrix
essMat=newRot*newSkew;
fundMat=inv(KmatrixTwo')*essMat*inv(KmatrixOne)

return