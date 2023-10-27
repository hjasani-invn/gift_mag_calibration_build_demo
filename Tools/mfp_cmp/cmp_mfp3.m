function cmp_mfp3(fName1, fName2)
    close all;
    % reads binary file *.mfp3

%     % Regus    
%     mfSizeX=24;% m
%     mfSizeY=15;% m
%     mfCellSize=1;% m

    % Model
    mfSizeX = 30; % m
    mfSizeY = 30; % m
    mfCellSize = 1; % m
    
    mfFloor = 0;

    [avX1, avY1, avZ1, sigX1, sigY1, sigZ1]  = LoadMfp3(fName1, mfSizeX,mfSizeY, mfCellSize, mfFloor);
    [avX2, avY2, avZ2, sigX2, sigY2, sigZ2]  = LoadMfp3(fName2, mfSizeX,mfSizeY, mfCellSize, mfFloor);
    
%     figure; pcolor(avX1'); axis equal; axis([1 mfSizeX 1 mfSizeY]);  %savefig('model_mfp_x.fig');
%     figure; pcolor(avY1'); axis equal; axis([1 mfSizeX 1 mfSizeY]);  %savefig('model_mfp_y.fig');
%     figure; pcolor(avZ1'); axis equal; axis([1 mfSizeX 1 mfSizeY]);  %savefig('model_mfp_z.fig');

%     figure; pcolor(sigX1'); axis equal; axis([1 mfSizeX 1 mfSizeY]); caxis([0 25]);   %savefig('model_mfp_dx.fig');
%     figure; pcolor(sigY1'); axis equal; axis([1 mfSizeX 1 mfSizeY]); caxis([0 25]);   %savefig('model_mfp_dy.fig');
%     figure; pcolor(sigZ1'); axis equal; axis([1 mfSizeX 1 mfSizeY]); caxis([0 25]);   %savefig('model_mfp_dz.fig');

%     figure; pcolor(avX2'); axis equal; axis([1 mfSizeX 1 mfSizeY]);  %savefig(strcat(fig_name,'_mfp_x.fig'));
%     figure; pcolor(avY2'); axis equal; axis([1 mfSizeX 1 mfSizeY]);  %savefig(strcat(fig_name,'_mfp_y.fig'));
%     figure; pcolor(avZ2'); axis equal; axis([1 mfSizeX 1 mfSizeY]);  %savefig(strcat(fig_name,'_mfp_z.fig'));

%     figure; pcolor(sigX2'); axis equal; axis([1 mfSizeX 1 mfSizeY]); caxis([0 25]);  %savefig(strcat(fig_name,'_mfp_dx.fig'));
%     figure; pcolor(sigY2'); axis equal; axis([1 mfSizeX 1 mfSizeY]); caxis([0 25]);  %savefig(strcat(fig_name,'_mfp_dy.fig'));
%     figure; pcolor(sigZ2'); axis equal; axis([1 mfSizeX 1 mfSizeY]); caxis([0 25]);  %savefig(strcat(fig_name,'_mfp_dz.fig'));

    figure; pcolor( abs(avX2'-avX1') ); axis equal; axis([1 mfSizeX 1 mfSizeY]);  caxis([-5 5]);    %savefig(strcat(fig_name,'_dmfp_x.fig'));
    figure; pcolor( abs(avY2'-avY1') ); axis equal; axis([1 mfSizeX 1 mfSizeY]);  caxis([-5 5]);    %savefig(strcat(fig_name,'_dmfp_y.fig'));
    figure; pcolor( abs(avZ2'-avZ1') ); axis equal; axis([1 mfSizeX 1 mfSizeY]);  caxis([-5 5]);    %savefig(strcat(fig_name,'_dmfp_z.fig'));
 
%     figure; pcolor(sigX2'-sigX1'); axis equal; axis([1 mfSizeX 1 mfSizeY]);  caxis([0 15]);  %savefig(strcat(fig_name,'_dmfp_dx.fig'));
%     figure; pcolor(sigY2'-sigY1'); axis equal; axis([1 mfSizeX 1 mfSizeY]);  caxis([0 15]);  %savefig(strcat(fig_name,'_dmfp_dy.fig'));
%     figure; pcolor(sigZ2'-sigZ1'); axis equal; axis([1 mfSizeX 1 mfSizeY]);  caxis([0 15]);  %savefig(strcat(fig_name,'_dmfp_dz.fig'));
    
    dmx = reshape(avX2'-avX1', 1, []);
    dmy = reshape(avY2'-avY1', 1, []);
    dmz = reshape(avZ2'-avZ1', 1, []);
    
    sprintf('mean FP error  %f %f %f', mean(dmx), mean(dmy), mean(dmz))
    sprintf('std FP error  %f %f %f', std(dmx), std(dmy), std(dmz))
    
    dsx = reshape(sigX2'-sigX1', 1, []);
    dsy = reshape(sigY2'-sigY1', 1, []);
    dsz = reshape(sigZ2'-sigZ1', 1, []);
    
    sprintf('mean FP disp error  %f %f %f', mean(abs(dsx)), mean(abs(dsy)), mean(abs(dsz)))
    
%     close all;
    
%     dmx = avX2'-avX1'
%     dmy = avY2'-avY1'
%     dmz = avZ2'-avZ1'
%     mean(reshape(dmx,1,[]))
%     std(reshape(dmx,1,[]))
%     mean(reshape(dmy,1,[]))
%     std(reshape(dmy,1,[]))
%     mean(reshape(dmz,1,[]))
%     std(reshape(dmz,1,[]))
%     
%     dmx(4,15) = 0;
%     dmx(5,22) = 0;
%     dmy(4,15) = 0;
%     dmy(5,22) = 0;
%     dmz(4,15) = 0;
%     dmz(5,22) = 0;
% 
%     'clean'
%     mean(reshape(dmx,1,[]))
%     std(reshape(dmx,1,[]))
%     mean(reshape(dmy,1,[]))
%     std(reshape(dmy,1,[]))
%     mean(reshape(dmz,1,[]))
%     std(reshape(dmz,1,[]))

end

function [avX, avY, avZ, sigX, sigY, sigZ]  = LoadMfp3(fName, mfSizeX,mfSizeY, mfCellSize, mfFloor)

    floorCells = mfSizeX*mfSizeY/mfCellSize/mfCellSize;
    skipBytes = mfFloor*mfSizeX*mfSizeY/mfCellSize/mfCellSize*8*3*4;

    numX=int32(mfSizeX/mfCellSize);
    numY=int32(mfSizeY/mfCellSize);
    avX=zeros(numX,numY);
    avY=zeros(numX,numY);
    avZ=zeros(numX,numY);
    sigX=zeros(numX,numY);
    sigY=zeros(numX,numY);
    sigZ=zeros(numX,numY);


    fid=fopen(fName,'r');
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
end

