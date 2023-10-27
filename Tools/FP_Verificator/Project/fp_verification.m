function coverage_proc=fp_verification(filename,min_meas_num,sellsize)

formatSpec = '%f%f%f';
delimiter = ' ';
fileID = fopen(filename,'r');
dataArray = textscan(fileID, formatSpec, 'EmptyValue' ,NaN, 'ReturnOnError', false);
fclose(fileID);   
coord_data2=[dataArray{1:3}];
%xg=coord_data2(:,3);
%yg=coord_data2(:,4);
coord_data(1,:)=coord_data2(1,:);
%num2=length(xg);

coord_data=coord_data2;

x=coord_data(:,1);
y=coord_data(:,2);
s=coord_data(:,3);
num=length(s);
coverage_proc=0;
ind=find(s>=min_meas_num);
if ~isempty(ind)
    coverage_proc=length(ind)/num*100;
end;

% heatmap
c=zeros(num,3);
for k=1:num 
    if s(k)<min_meas_num
       c(k,:)= [0 0 0];%black
    else
        c(k,:)=[0 1 0];%'green'
    end;

end;
figure;scatter(x,y,40,c,'s','filled');grid on;
title(['Coverage: ',num2str(coverage_proc,'%5.1f'),'%']); 
end

