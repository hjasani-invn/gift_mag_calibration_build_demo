function mg_export(fname, mg, sig, cellsize)
    fid = fopen(fname, 'w');

    if (fid == 0)
        error('cant open file: %s', fname);
    end
    
    Yi=1/2:1:size(mg,1)-1/2;
    Xi=1/2:1:size(mg,2)-1/2;
 

    
    matrix_sig = ((size(mg,1)==size(sig,1))&& (size(mg,2)==size(sig,2)));
    dims = size(mg);
    

    idx_i = cellsize/2:cellsize:dims(1)-cellsize/2;
    idx_j = cellsize/2:cellsize:dims(2)-cellsize/2;
    
    for i = idx_i
        A = zeros(24,1);
        
        pos = i*ones(length(idx_j),2);
        mg_i = zeros(length(idx_j),3);
        
        pos(:,2)=idx_j;
        
        mg_i(:,1) = interp2(Xi, Yi, mg(:,:,1), pos(:,2), pos(:,1),'spline',0);
        mg_i(:,2) = interp2(Xi, Yi, mg(:,:,2), pos(:,2), pos(:,1),'spline',0);
        mg_i(:,3) = interp2(Xi, Yi, mg(:,:,3), pos(:,2), pos(:,1),'spline',0);
        jj=0;
        for j = idx_j
            jj = jj+1;
            for k=1:3
                A((k-1)*8+1) = mg_i(jj,k);
                if (matrix_sig==1)
                    A((k-1)*8+2) = sig(floor(i+1),floor(j+1));
                else
                    A((k-1)*8+2) = sig;
                end
            end;
            fwrite(fid, A, 'float');
        end
    end
    
%     for i=1:dims(1)
%         A = zeros(24,1);
%         for j=1:dims(2)
%             for k=1:3
%                 A((k-1)*8+1) = mg(i, j, k);
%                 if (matrix_sig==1)
%                     A((k-1)*8+2) = sig(i,j);
%                 else
%                     A((k-1)*8+2) = sig;
%                 end
%             end;
%             fwrite(fid, A, 'float');
%         end
%     end
    
    fclose(fid);
end