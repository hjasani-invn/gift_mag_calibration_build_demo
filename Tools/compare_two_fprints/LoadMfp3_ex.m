function [map, mag_sigma] = LoadMfp3_ex(fName, mfSizeX, mfSizeY, mfCellSize, floorNumber)

numX = ceil(mfSizeX/mfCellSize);
numY = ceil(mfSizeY/mfCellSize);

floorCells = numX*numY;
skipBytes = (floorNumber-1)*numX*numY*8*3*4;

avX=zeros(numX,numY);
avY=zeros(numX,numY);
avZ=zeros(numX,numY);
sigX=zeros(numX,numY);
sigY=zeros(numX,numY);
sigZ=zeros(numX,numY);

aPortion=single(zeros(1,24));
fid=fopen(fName,'r');

fseek(fid,skipBytes,'bof');
iX=1;
iY=1;
i=0;
while (~feof(fid) && i<floorCells)
    aPortion=fread(fid,24,'float');
    if length(aPortion)<24
        break;
    end    
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
    end
    i = i +1;
    
end

magX=avX';
magY=avY';
magZ=avZ';

map(:,:,1) = magX;
map(:,:,2) = magY;
map(:,:,3) = magZ;

mag_sigma(:,:,1) = sigX';
mag_sigma(:,:,2) = sigY';
mag_sigma(:,:,3) = sigZ';

fclose(fid);

end