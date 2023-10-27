% function WFP_uncertainty estimates predicted or expected uncertainy of
% WFP-based positioning based on properties of WFP map
% Parameters:
% fileMFP3 is WFP map in wifi3 format
% dimX and dimY are WFP horizontal and vertical dimensions in meters 
% floor is a number of floor starting from 0
% cellsize is  WFP cell size in meters
% Return values:
% WFP_Unc is 2D array of WFP uncertainties per cell
% mean_unc is an average uncertainty
% reslt=1 if OK and 0 otherwise (usually if wifi3 file has not been found)

function [WFP_Unc, mean_unc, result] = WFP_uncertainty(fileWFP3, dim_X, dim_Y, floor, cellsize)
result=1;
if ~exist(fileWFP3,'file')
   result=0;
   return;
end


init_unc=-5;

delimiterIn = '\t';

% Read WFP3 file
fileID = fopen(fileWFP3,'r');
if fileID==-1
    result=0;
    return;
else
    B = importdata(fileWFP3,delimiterIn);
    WFP=[];
    ap_index=[];
    xy_index=[];
    k=0;
    curfloor=-1;
    for i=1:length(B)
        c=B(i);
        newStr = split(c);
        if strcmp(newStr(1),'POINT')
            cur_x=str2double(newStr(2));
            cur_y=str2double(newStr(3));
            curfloor=str2double(newStr(4));                  
        end;
        if curfloor==floor
            if strcmp(newStr(1),'AP_MAC')
                alpha1=str2double(newStr(3));
                mean1=str2double(newStr(4));
                std1=str2double(newStr(5)); 
                alpha2=str2double(newStr(6));
                mean2=str2double(newStr(7));
                std2=str2double(newStr(8));

                %if mean1==-100 && alpha2>0.5 && mean2<0
                if mean1==-100 && alpha2>0.5 && mean2<0 && mean2>-80
                    av_mean=mean2;
                %elseif mean2==-100 && alpha1>0.5 && mean1<0
                elseif mean2==-100 && alpha1>0.5 && mean1<0 && mean1>-80
                    av_mean=mean1;
                elseif  mean1>-80 && mean2>-80 && mean1<0 && mean2<0
                    av_mean=alpha1*mean1+alpha2*mean2;
                else
                    av_mean=-100;
                end;
                av_std=7.5;%sqrt(av_std^2+2*25);
                if av_mean>-90
                    k=k+1;
                    WFP(k,1)=cur_x;
                    WFP(k,2)=cur_y;
                    WFP(k,3)=curfloor;
                    WFP(k,4)=str2double(newStr(2)); %BSSID
                    WFP(k,5)=av_mean;
                    WFP(k,6)=av_std;
                    ap_index=[ap_index,AP_search_key(cur_x,cur_y,newStr(2))];
                    xy_index=[xy_index,XY_search_key(cur_x,cur_y)];
                end;           
            end;
        end
    end;
    if isempty(WFP)
        result=0;
        return;
    end;
   
    fclose(fileID);
end;
clear B;

maxX=max(WFP(:,1));
maxY=max(WFP(:,2));
dimX=round((dim_X+cellsize/2)/cellsize);
dimY=round((dim_Y+cellsize/2)/cellsize);

macc=init_unc*ones(dimY,dimX);
macc_smooth=init_unc*ones(dimY,dimX);


% WFP uncertainty


accdata(1)=0;
for i=1:dimY
    for j=1:dimX
    Cxx=zeros(2,2);
    x=double(j)*cellsize-cellsize/2;
    y=double(i)*cellsize-cellsize/2;
    xy_key=XY_search_key(x,y);
    curr_cell=find(strcmp(xy_index(:),xy_key));
    if ~isempty(curr_cell)
        xm1=x-cellsize;
        xp1=x+cellsize;
        ym1=y-cellsize;
        yp1=y+cellsize;
        % gradient for every ap
        hor_dev=[];
        ver_dev=[];
        sig2=[];
        a=WFP(curr_cell,:);
        [~,I]=sort(a(:,5),'descend');
        sortedWFP=WFP(curr_cell(I),:);
        %len=min(length(curr_cell),5);
        len=length(curr_cell);
        for k=1:len
            %[rssi_curr, sig_curr]=get_rssi_WFP(WFP(curr_cell(k),:));              
            %cur_ap=num2str(WFP(curr_cell(k),4));
            [rssi_curr, sig_curr]=get_rssi_WFP(sortedWFP(k,:));            
            cur_ap=num2str(sortedWFP(k,4));
            %[rssi_curr, sig_curr]=get_rssi_WiFiGrid(wgrid,ap_index_grid,x,y,cur_ap);
            xm1_key=AP_search_key(xm1,y,cur_ap);
            xp1_key=AP_search_key(xp1,y,cur_ap);
            ym1_key=AP_search_key(x,ym1,cur_ap);
            yp1_key=AP_search_key(x,yp1,cur_ap);
            cell_xm1=find(strcmp(ap_index(:),xm1_key));
            cell_xp1=find(strcmp(ap_index(:),xp1_key));
            cell_ym1=find(strcmp(ap_index(:),ym1_key));
            cell_yp1=find(strcmp(ap_index(:),yp1_key));
            
            count_hor=0;
            hor_dev_ap=0;
            if length(cell_xm1)==1
                [rssi, sig]=get_rssi_WFP(WFP(cell_xm1,:));
%                [rssi, sig]=get_rssi_WiFiGrid(wgrid,ap_index_grid,xm1,y,cur_ap);
                hor_dev_ap=hor_dev_ap+(rssi_curr-rssi);
                count_hor=count_hor+1;
            end;
            if length(cell_xp1)==1
                [rssi, sig]=get_rssi_WFP(WFP(cell_xp1,:));
%                [rssi, sig]=get_rssi_WiFiGrid(wgrid,ap_index_grid,xp1,y,cur_ap);
                hor_dev_ap=hor_dev_ap+(rssi-rssi_curr);
                count_hor=count_hor+1;
            end;
            count_ver=0;
            ver_dev_ap=0;
            if length(cell_ym1)==1
                [rssi, sig]=get_rssi_WFP(WFP(cell_ym1,:));
%                [rssi, sig]=get_rssi_WiFiGrid(wgrid,ap_index_grid,x,ym1,cur_ap);
                ver_dev_ap=ver_dev_ap+(rssi_curr-rssi);
                count_ver=count_ver+1;
            end;
            if length(cell_yp1)==1
                [rssi, sig]=get_rssi_WFP(WFP(cell_yp1,:));
%                [rssi, sig]=get_rssi_WiFiGrid(wgrid,ap_index_grid,x,yp1,cur_ap);
                ver_dev_ap=ver_dev_ap+(rssi-rssi_curr);
                count_ver=count_ver+1;
            end;
            if (count_hor>0)&&(count_ver>0)
                hor_dev=[hor_dev;hor_dev_ap/count_hor];
                ver_dev=[ver_dev;ver_dev_ap/count_ver];               
            elseif (count_hor>0)&&(count_ver==0)
                hor_dev=[hor_dev;hor_dev_ap/count_hor];
                ver_dev=[ver_dev;0];
            elseif (count_hor==0)&&(count_ver>0)
                hor_dev=[hor_dev;0];
                ver_dev=[ver_dev;ver_dev_ap/count_ver];
            end;
            if (count_hor>0)||(count_ver>0)
                sig2=[sig2;sig_curr*sig_curr];
            end;
        end; % k - APs
        hor_0=length(find(hor_dev==0));
        ver_0=length(find(ver_dev==0));
        len_dev=length(hor_dev);
        if len_dev>=2
            if len_dev>hor_0&&len_dev>ver_0
                DF=[hor_dev,ver_dev]/cellsize;
                Cmeas=diag(sig2);            
                if det(Cmeas)>0.00000001
                        M=DF'*inv(Cmeas)*DF;
                        if det(M)>0.00000001
                            Cxx=inv(M);
                            acc=sqrt(max(eig(Cxx)));
                            macc(i,j)=acc;
                            %acc=sqrt(Cxx(1,1)^2+Cxx(2,2)^2);
                         end;   
                end;
            elseif len_dev>hor_0&&len_dev==ver_0
                r=((hor_dev/cellsize).^2)./sig2;
                acc=1/sqrt(sum(r)); 
                macc(i,j)=acc;
            elseif len_dev==hor_0&&len_dev>ver_0
                r=((ver_dev/cellsize).^2)./sig2;
                acc=1/sqrt(sum(r));  
                macc(i,j)=acc;
            end;
        end;
    end; % ~isempty(curr_cell)
    
    
    end;%j
end;%i
WFP_Unc=macc;

% 2D filter (gaussian)
hsize=1;
%hsig=2.5;
hsig=cellsize;
hsum=0;
for k=-hsize:hsize
    for m=-hsize:hsize
        b=exp(-(k^2+m^2)*cellsize^2/2/hsig^2);
        hsum=hsum+b;
        h(k+hsize+1,m+hsize+1)=b;
    end;
end;
h=h/hsum;  

for i=1:dimY
    for j=1:dimX
        fres=0;
        hsum=0;
        for k=1:hsize*2+1
            for m=1:hsize*2+1                
                ih=i-k+2;
                jh=j-m+2;
                if ih>0&&jh>0&&ih<=dimY&&jh<=dimX
                    if macc(ih,jh)>0
                        fres=fres+macc(ih,jh)*h(k,m);
                        hsum=hsum+h(k,m);                
                    end;
                end;                
            end;
        end;
        if macc(i,j)>0
            macc_smooth(i,j)=fres/hsum;        
        end;
    end;
end;    

mean_unc=mean(macc(find(macc>0)));
h=figure;p=pcolor(macc(:,:));axis equal tight;colormap jet;
title({['WFP mean uncertainty (m): ',num2str(mean_unc,'%5.1f')]});grid on;
colorbar('Ticks',[-5,0,5,10,15,20,25,30,35,40,45,50],...
         'TickLabels',{'Unsurveyed','0','5 m','10 m','15 m','20 m','25 m','30 m','35 m','40 m','45 m','50 m'});caxis([-5 50]);
     
h1=figure;p=pcolor(macc_smooth(:,:));axis equal tight;colormap jet;
title({['WFP smooth uncertainty (m): ',num2str(mean_unc,'%5.1f')]});grid on;
colorbar('Ticks',[-5,0,5,10,15,20,25,30,35,40,45,50],...
         'TickLabels',{'Unsurveyed','0','5 m','10 m','15 m','20 m','25 m','30 m','35 m','40 m','45 m','50 m'});caxis([-5 50]);     
end

function key = AP_search_key(x,y,AP)
sX=num2str(x*10);
sY=num2str(y*10);
key = string(strcat(sX,sY,AP));
end

function key = XY_search_key(x,y)
sX=num2str(x*10);
sY=num2str(y*10);
key = string(strcat(sX,sY));
end

function [rssi, sig]=get_rssi_WFP(wfp)
rssi=wfp(5);
sig=wfp(6);
% rssi=wfp(5)*wfp(6)+wfp(8)*wfp(9);
% sig=wfp(5)*wfp(7)+wfp(8)*wfp(10);
end

function [rssi, sig]=get_rssi_WiFiGrid(wgrid,ap_index_grid,x,y,AP)
key=AP_search_key(x,y,AP);
c= find(strcmp(ap_index_grid(:),key));
r=wgrid(c,3);
len=length(r);
if ~isempty(r)&&len>=10
    rssi=mean(r);
    sig=std(r);
else
    rssi=-100;
    sig=100;
end;
end