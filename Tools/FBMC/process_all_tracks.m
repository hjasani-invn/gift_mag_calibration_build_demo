function process_all_tracks
addpath ('file_lib');

path = 'C:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\FP_builder\Data_for_mag_calibration_2_dat_bin\';

FolderInfo = dir(path);
%figure;
file_stat_name = strcat(path, 'mag_bias_by_spectr-5.csv');
fstat = fopen(file_stat_name,'wt');
for i = 3:size(FolderInfo, 1)
    if (FolderInfo(i).isdir)
        path_l1 = strcat(path, FolderInfo(i).name, '\');
        FolderInfo_l1 = dir(path_l1);
        for j = 3:size(FolderInfo_l1, 1)
            if (FolderInfo_l1(j).isdir)
                path_l2 = strcat(path_l1, FolderInfo_l1(j).name, '\');
                %file_name = strcat(path_l2, 'irl_data_uncorrected.csv')
                file_name = strcat(path_l2, 'irl_mag_data.csv');                
                mgbias_file_name = strcat(path_l2, 'default.mbias');
                [m2_level, m2_bias, m2_cov_matrix] = LoadMgbiasFile(mgbias_file_name);
                %magout_file_name = strcat(path_l2, 'mag_out.txt');
                magout_file_name = strcat(path_l2, 'mag_out_by_fbmc-4.txt');
                [jo_level, jo_bias, jo_cov_matrix] = LoadMagOutFile(magout_file_name);
                fin = fopen(file_name,'rt');
                if (fin > 0)
                    fclose(fin);
                    [m_bias_est, m_bias_cov, m_level_est] = mag_bias_by_spectr_by_irl(file_name);
                    bias_file_name = strcat(path_l2, 'mag_out_by_fbmc-5.txt');
                    fout = fopen(bias_file_name,'wt');
                    fprintf(fout, '%d,%12.3f,%12.3f,%12.3f,%12.3f,%12.3f,%12.3f',m_level_est, m_bias_est(1), m_bias_est(2), m_bias_est(3), m_bias_cov(1), m_bias_cov(5), m_bias_cov(9));
                    
                    fprintf(fstat, '%s,%5d,%12.3f,%12.3f,%12.3f,%12.3f,%12.3f,%12.3f',path_l2, m_level_est, m_bias_est(1), m_bias_est(2), m_bias_est(3), m_bias_cov(1), m_bias_cov(5), m_bias_cov(9));
                    %fprintf(fstat, ';    %d; %.3f; %.3f; %.3f;  %.3f; %.3f; %.3f',m2_level, m2_bias(1), m2_bias(2), m2_bias(3), m2_cov_matrix(1,1), m2_cov_matrix(2,2), m2_cov_matrix(3,3));
                    %fprintf(fstat, ';    %d; %.3f; %.3f; %.3f;  %.3f; %.3f; %.3f',jo_level, jo_bias(1), jo_bias(2), jo_bias(3), jo_cov_matrix(1,1), jo_cov_matrix(2,2), jo_cov_matrix(3,3));
                    fprintf(fstat, '\n');

                    fclose(fout);
                end
            end
        end
    end
end
fclose(fstat);
end

