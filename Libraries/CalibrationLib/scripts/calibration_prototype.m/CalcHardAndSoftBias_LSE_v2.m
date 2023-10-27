function [bias] = CalcHardAndSoftBias_GM_v2(mg_data)
% hard and soft iron calibration

% best is a vector for hard iron compensation
% S_1 is a matrix for soft iron compensation

%Acquisition of a set of points from file

%F = importdata('C:\Work\Calibration\Gimble\2021_03_15_21_26_15_mag_bilda_645_half.csv',',');
%U2 = raw_mg';
U = mg_data';
% % Averaging
% len2=length(U2);
% avint=20;
% maxi=floor(len2/avint)-1;
% U=[];
% for i=1:maxi
%    kb=1+(i-1)*avint;
%    ke=kb+avint;
%    por=U2(:,kb:ke);
%    vav=mean(por,2);
%    vstd=std(por,0,2);
%        if max(vstd)<0.5
%            U=[U vav];
%        end
% end

%figure;scatter3(U(1,:),U(2,:),U(3,:),10,'filled');axis equal;grid on;title('Before calibration');

len=length(U);
magmag=[];
for i=1:len
    h=U(:,i);
    magmag=[magmag norm(h)];
end
%figure;plot(magmag,'.');grid on;title('Magnitude before calibration');
% Determination b and S
% Construct design matrix
D=[];
X=[];
Y=[];
for i=1:len
   x=U(1,i);
   y=U(2,i);
   z=U(3,i);
   d=[x^2,y^2,z^2,2*y*z,2*x*z,2*x*y,2*x,2*y,2*z,1];   
   D=[D;d];
   X=[X;[x,y,z,1]];
   Y=[Y;x^2+y^2+z^2];
end
% Bias according to AN4246 (for comparison)
bet=inv(X'*X)*X'*Y;
best4246=bet(1:3)/2;
BTB4246=bet(4)+best4246'*best4246;
% Scatter matrix
P=D'*D;
pn = norm(P,'fro');
%pn = norm(P);
%P=P/pn;
% Split P
S11=P(1:6,1:6);
S22=P(7:10,7:10);
S12=P(1:6,7:10);
%S21=P(7:10,1:6);
S22_1=inv(S22);
S_1=eye(3,3);
ql=0;
qh=0;
q=100;
%Iterative ellipsoid fitting
[a,Q, cnd]=elfit(S11, S22_1, S12, q);
wc=1;
if cnd>0
    wc=0;
else
   if q>3
       while wc==1
           q=q/2;
           ql=max(q,3);
           qh=2*q;
           q=(ql+qh)/2;
           [a,Q, cnd]=elfit(S11, S22_1, S12, q);
           if cnd>0
              ql=q;
           else
               qh=q;
           end
           if abs(qh-ql)<0.1
               break;
           end
       end
   end
end

% Estimate hard iron compensation vector (bias)
best=-inv(Q)*a(7:9);

% Estimate of B'B
BTB=abs(best'*Q*best-a(10));
%Estimate correction
coef = sqrt(BTB4246/BTB);
[Xq,Yq,Zq] = svd(Q);
Ys=sqrt(Yq);
% Soft iron compensation matrix
S_1=Xq*Ys*Xq'*coef;

bias = [best;  S_1(:,1); S_1(:,2); S_1(:,3)];

% Calibration
% Uc=[];
% magn=[];
% for i=1:len
%     h=U(:,i);
%     hc=S_1*(h-best);    
%     Uc = [Uc hc];
%     magn=[magn norm(hc)];
% end
% 
% figure;scatter3(Uc(1,:),Uc(2,:),Uc(3,:),10,'filled');axis equal;grid on;title('After calibration');
% figure;plot(magn);grid on;title(['Magnitude: mean ',num2str(mean(magn),'%8.4f'),' STD ',num2str(std(magn),'%10.5e'),' uT']);

end 

%----------------- fitting of ellipsoid ----------------------------------
function [a,Q,cnd]=elfit(S11, S22_1, S12, q)
    
C1=[-1 q/2-1 q/2-1 0 0 0;...
        q/2-1 -1 q/2-1 0 0 0;...
        q/2-1 q/2-1 -1 0 0 0;...
        0 0 0 -q 0 0;...
        0 0 0 0 -q 0;...
        0 0 0 0 0 -q];
    C_1=inv(C1);
    M=C_1*(S11-S12*S22_1*S12');
    [V,E] = eig(M);
    k0=0;
    % find the only positive eigenvalue
    for k=1:6
       lam=E(k,k);
       if lam>0 && isreal(lam)
          k0=k;
          break;     
       end
    end
    if k0==0
        a=eye(10,1);
        Q=eye(3,3);
        cnd=-1;
        return;
    end
    % Coresponding eigen vector
    v=V(:,k0);
    % Lagrange multiplier
    mu=sqrt(1/(v'*C1*v));
    a1=v*mu;
    a2=-S22_1*S12'*a1;
    a=[a1;a2];
    % (a; b; c; f; g; h; p; q; r; d)
    % I = a + b + c
    % J = ab + bc + ac - f^2 - g^2 - h^2
    I=a(1)+a(2)+a(3);
    J=a(1)*a(2)+a(2)*a(3)+a(1)*a(3)-a(4)^2-a(5)^2-a(6)^2;
    Q=[a(1),a(6),a(5);a(6),a(2),a(4);a(5),a(4),a(3)];
    if J>0 && I*det(Q)>0 
        cnd=1; % this is an ellipsoid
    else
        cnd=-1; % this is not
    end
    
end
