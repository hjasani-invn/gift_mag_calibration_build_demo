function save_mfp3(fname, mg, sig)

    fid = fopen(fname, 'w');

    if (fid == 0)
        error('cant open file: %s', fname);
    else
        dims = size(mg);
        if (size(dims,2) == 4)
            SaveMultifloorMFP(fid, mg, sig);
        elseif (size(dims,2) == 3)
            SaveSinglFloorMFP(fid, mg, sig);
        else
            error('MFP DB format error');
        end 
        fclose(fid);        
    end

end

function SaveMultifloorMFP(fid, mg, sig)
    matrix_sig = ((size(mg,1)==size(sig,1))&& (size(mg,2)==size(sig,2)));
    dims = size(mg);
    for fl = 1:dims(1)
        for i=1:dims(2)
            A = zeros(24,1);
            for j=1:dims(3)
                for k=1:3
                    A((k-1)*8+1) = mg(fl, i, j, k);
                    if (matrix_sig==1)
                        A((k-1)*8+2) = sig(fl, i,j,k);
                    else
                        A((k-1)*8+2) = sig;
                    end
                    A((k-1)*8+3) = 1;
                end
                fwrite(fid, A, 'float');
            end
        end
    end
end

function SaveSinglFloorMFP(fid, mg, sig)
    matrix_sig = ((size(mg,1)==size(sig,1))&& (size(mg,2)==size(sig,2)));
    dims = size(mg);
    for i=1:dims(1)
        A = zeros(24,1);
        for j=1:dims(2)
            for k=1:3
                A((k-1)*8+1) = mg(i, j, k);
                if (matrix_sig==1)
                    A((k-1)*8+2) = sig(i,j,k);
                else
                    A((k-1)*8+2) = sig;
                end
                A((k-1)*8+3) = 1;
            end
            fwrite(fid, A, 'float');
        end
    end
end