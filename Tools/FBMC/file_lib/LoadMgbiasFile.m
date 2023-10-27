
function [clb_level, mgbias, cov_matrix] = LoadMgbiasFile(file_name)
    %set default values
    clb_level = 0;
    mgbias = [1e6 1e6 1e6];
    cov_matrix(3,3) = 1e6;
    cov_matrix(2,2) = 1e6;
    cov_matrix(1,1) = 1e6;

    fin = fopen(file_name, 'rt');
    if (fin > 0)
        data = fscanf(fin,'%f');
        if (size(data,1) == 7)
            if ((norm(data(2:4)) > 0.001) && (norm(data(2:4)) < 9e5) && ...
                (norm(data(5:7)) > 0.001) && (norm(data(5:7)) < 9e5))
                clb_level = 3;
                mgbias = [data(2) data(3) data(4)];
                cov_matrix(3,3) = data(7);
                cov_matrix(2,2) = data(6);
                cov_matrix(1,1) = data(5);
            else
                disp('WARNRING: Incorrect mgbis file format or data')
            end
            fclose(fin);
        else
            disp('WARNRING: No mgbias file found')
        end
    end
end
