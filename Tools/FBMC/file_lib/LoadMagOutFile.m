function [clb_level, mgbias, cov_matrix] = LoadMagOutFile(file_name)
    %set default values
    clb_level = 0;
    mgbias = [1e6 1e6 1e6];
    cov_matrix(3,3) = 1e6;
    cov_matrix(2,2) = 1e6;
    cov_matrix(1,1) = 1e6;

    fin = fopen(file_name, 'rt');
    if (fin > 0)
        data = fscanf(fin,'%f,', 13);
        if (size(data,1) == 13)
            clb_level = data(1);
            mgbias = [data(2) data(3) data(4)];
            cov_matrix(3,3) = data(13); cov_matrix(3,2) = data(12); cov_matrix(3,1) = data(11);
            cov_matrix(2,3) = data(10); cov_matrix(2,2) = data(9);  cov_matrix(2,1) = data(8);
            cov_matrix(1,3) = data(7);  cov_matrix(1,2) = data(6);  cov_matrix(1,1) = data(5);
        else
            disp('WARNRING: Incorrect mag_out file format')
        end
        fclose(fin);
    else
        disp('WARNRING: No mgbias mag_out found')
    end
end
