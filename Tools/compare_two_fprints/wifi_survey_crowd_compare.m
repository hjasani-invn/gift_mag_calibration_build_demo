wifi_file='c:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\venues\Bridgestone_hikone\compare_wifi_results\a8e7_vs_1c99_all.csv';
delimiterIn=',';
A = importdata(wifi_file,delimiterIn);
len=size(A,1);
C=zeros(len,5);
for k=1:len
    C(k,1)=A(k,1);%X
    C(k,2)=A(k,2);%Y
    C(k,3)=A(k,4);
    d1=0;
    d2=0;
    d=0;
    v1=-100;
    w1s=A(k,6);
    m1s=A(k,7);
    w1c=A(k,8);
    m1c=A(k,9);
    w2s=A(k,10);
    m2s=A(k,11);
    w2c=A(k,12);
    m2c=A(k,13);
    
    if w1s>0.1 && m1s>-100 && w1c>0.1 && m1c>-100 && (w2s<0.1 || m2s==-100)  
        d=m1s-m1c;
        v1=m1s;
    elseif w2s>0.1 && m2s>-100 && w2c>0.1 && m2c>-100 && (w1s<0.1 || m1s==-100)
        d=m2s-m2c;
        v1=m2s;
    elseif w1s>0.1 && m1s>-100 && w2c>0.1 && m2c>-100 && (w1c<0.1 || m1c==-100)
        d=m1s-m2c;
        v1=m1s;
    elseif w2s>0.1 && m2s>-100 && w1c>0.1 && m1c>-100 && (w2c<0.1 || m2c==-100)
        d=m2s-m1c;
        v1=m2s;
    elseif w1s>0.1 && m1s>-100 && w1c>0.1 && m1c>-100 && w2s>0.1 && m2s>-100 && w2c>0.1 && m2c>-100
        v1=w1s*m1s+w2s*m2s;
        v2=w1c*m1c+w2c*m2c;
        d=v1-v2;
    else
        d=0;
        v1=-100;
    end    
    C(k,4)=d;
    C(k,5)=v1;
end
%C(:,4)=abs(C(:,4));

% Create a list of BSSID
APlist=uint64(C(1,3));
x=A(1,1);
y=A(1,2);
for k=2:len
   if A(k,1)==x && A(k,2)==y
       APlist=[APlist;uint64(C(k,3))];
   else
       break;
   end
   x=A(k,1);
   y=A(k,2);
end

len_list=size(APlist,1);
av_err=zeros(len_list,1);
av_strength=zeros(len_list,1);
for k=1:len_list
   ind=find(uint64(C(:,3))==APlist(k));
   Ci=C(ind,:);
   av_err(k)=mean(Ci(:,4));
   av_strength(k)=mean(Ci(:,5));
end
av_err
s = '============'
av_strength
s = '============'



