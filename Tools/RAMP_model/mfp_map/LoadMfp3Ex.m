function [map, map_sigma] = LoadMfp3Ex(fName, mfSizeX, mfSizeY, mfCellSize)
%fName='t:\Frolov\roppongi.mfp3';


% mfSizeX=88;% m
% mfSizeY=87;% m
% mfCellSize=1;% m

% set current floor from 0 to max
mfFloor=0;

floorCells = mfSizeX*mfSizeY/mfCellSize/mfCellSize;
skipBytes = mfFloor*mfSizeX*mfSizeY/mfCellSize/mfCellSize*8*3*4;

% hfilter = fspecial('gaussian', [4 4], 0.1);

% MeasAcc=1.0; % uT
numX=int32(mfSizeX/mfCellSize);
numY=int32(mfSizeY/mfCellSize);
% mfIndAbs=zeros(numX,numY);
avX=zeros(numX,numY);
avY=zeros(numX,numY);
avZ=zeros(numX,numY);
sigX=zeros(numX,numY);
sigY=zeros(numX,numY);
sigZ=zeros(numX,numY);

% avGradX=zeros(numX,numY); 
% avGradY=zeros(numX,numY); 
% avGradXY=zeros(numX,numY); 
% avAcc=zeros(numX,numY); 

aPortion=single(zeros(1,24));
fid=fopen(fName,'r');
% 6-th floor for office!!!
fseek(fid,skipBytes,'bof');
iX=1;
iY=1;
i=0;
while (~feof(fid) && i<floorCells)
    aPortion=fread(fid,24,'float');
    if length(aPortion)<24
        break;
    end;    
    avX(iX,iY)=aPortion(1,1);
    sigX(iX,iY)=aPortion(2,1);
    avY(iX,iY)=aPortion(9,1);
    sigY(iX,iY)=aPortion(10,1);
    avZ(iX,iY)=aPortion(17,1);
    sigZ(iX,iY)=aPortion(18,1);
%    mfIndAbs(iX,iY)=sqrt(avX*avX+avY*avY+avZ*avZ);
    iX=iX+1;
    if iX>numX
        iX=1;
        iY=iY+1;
    end;
    i = i +1;
    
end;

fclose(fid);

magX=avX';
magY=avY';
magZ=avZ';

map(:,:,1) = magX;
map(:,:,2) = magY;
map(:,:,3) = magZ;

map_sigma(:,:,1) = sigX';
map_sigma(:,:,2) = sigY';
map_sigma(:,:,3) = sigZ';

end