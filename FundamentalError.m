%function [fund1Error,fund2Error] = FundamentalError(CamOneCoords,CamTwoCoords,calcMat,approxMat)
calcMat=fundMat;
approxMat=F;
CamOneCoords=CamOneCoord;
CamTwoCoords=CamTwoCoord;
%for the calculated fundamental matrix
x1=CamOneCoords(1,:)';
y1=CamOneCoords(2,:)';
x2=CamTwoCoords(1,:)';
y2=CamTwoCoords(2,:)';
rErrors=ones(39,1);
for i=1:39
    rLine=calcMat*[x1(i) y1(i) 1]';
    a=rLine(1);
    b=rLine(2);
    c=rLine(3);
    rErrors(i)=((a*x1(i)+b*y1(i)+c)^2)/(a^2+b^2);
end
lErrors=ones(39,1);
for i=1:39
    lLine=calcMat*[x2(i) y2(i) 1]';
    a=lLine(1);
    b=lLine(2);
    c=lLine(3);
    lErrors(i)=((a*x2(i)+b*y2(i)+c)^2)/(a^2+b^2);
end
fund1Error=(mean(lErrors)+mean(rErrors))/2


%for the approximated fundamental matrix
x1=CamOneCoords(1,:)';
y1=CamOneCoords(2,:)';
x2=CamTwoCoords(1,:)';
y2=CamTwoCoords(2,:)';
rErrors=ones(39,1);
for i=1:39
    rLine=approxMat*[x1(i) y1(i) 1]';
    a=rLine(1);
    b=rLine(2);
    c=rLine(3);
    rErrors(i)=((a*x1(i)+b*y1(i)+c)^2)/(a^2+b^2);
end
lErrors=ones(39,1);
for i=1:39
    lLine=approxMat*[x2(i) y2(i) 1]';
    a=lLine(1);
    b=lLine(2);
    c=lLine(3);
    lErrors(i)=((a*x2(i)+b*y2(i)+c)^2)/(a^2+b^2);
end
fund2Error=(mean(lErrors)+mean(rErrors))/2