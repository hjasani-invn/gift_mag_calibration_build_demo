function [m_bias_est, m_bias_cov, level] = mag_bias_by_spectr_with_irl_input(filename)
%addpath ('..\lib\plot_lib');
%filename = 'C:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Applications\fp_builder.console\FP_builder\m.txt'
%filename = 'C:\GitHub\Gift\Applications\fp_positioning.console\Project\tpn_converter_out_17.txt';
%filename = 'C:\Share\September_30_2016\android_pocket1\011\irl_data_uncorrected.csv';
data = dlmread(filename, ';', 0, 0);

%data = RejectStands(data);

L = size(data, 1);
if L > 0
%t = data(1:L,1);
dl = 1500;
m_udf = data(dl:L-dl+1,29:31)/10;
%m_udf = data;
%q_bf2mfp = [];
m_bias_0 = m_udf(1,:);

[m_bias_est, m_bias_cov, level] = LSE_bias_by_stectr_raw_module_crt_stoch_start (m_udf, m_bias_0);

m_bias_est
else
    m_bias_est = zeros(1,3) ; 
    m_bias_cov = zeros(1,9);
    level = 0;
end
end

function data = RejectStands(data)
    stride = data(:,26:27); 
    idx = find (stride(:,2) ~= 0);
    data  = data(idx,:);
    idx = find (stride(:,1) ~= 0);
    data  = data(idx,:);
end

function data = RejectMagOutliers(data)
    % empty
end

function mag = GetVectorMagnitude(v)
    mag = sqrt(v(:,1).*v(:,1) + v(:,2).*v(:,2) + v(:,3).*v(:,3));
end