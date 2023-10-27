% function create_map_ex1
clc;
clear all
close all

floor_number = 0; % default for singlefloors venues

interpolation_method = 'nearest';
%interpolation_method = 'liniar';

% model map
% x_sz_in = 80;
% y_sz_in = 80;
% cell_sz_in = 2;
% cell_sz_out = 1;
% fname = '2m_model_err_3ut';
venue_id = 'default shadow';

% Northland v2
if 0
    %venue_id = 'Northland';
    venue_id = 'Northland shadow';
    x_sz = 460;
    y_sz = 200;
    cell_sz = 2;
    floor_number = 2;
    fname = '\\cayyc-proj01\compute02\dchurikov\Tasks\MFP_interpolation\Fingerprints\Nortland\Maps\fpl_mag-cs1';
    %fname = '\\cayyc-proj01\compute02\dchurikov\Tasks\MFP_interpolation\Nortland\Maps\fpl_mag-ref';
    %fmask_name = '\\cayyc-proj01\compute02\dchurikov\Tasks\MFP_interpolation\Nortland\Maps\fpl_mag-ref.mfp3';
    fmask_name = '\\cayyc-proj01\compute02\dchurikov\Tasks\MFP_interpolation\Fingerprints\Nortland\Maps\fpl_mag-cs1.mfp3';
end

% % Schnucks
if 1
    %venue_id = 'Schnucks';
    venue_id = 'Schnucks shadow';

    x_sz = 140;
    y_sz = 85;
    cell_sz = 1;
    floor_number = 1;
    fname = '\\cayyc-proj01\compute02\dchurikov\Tasks\MFP_interpolation\Fingerprints\Schnucks\Maps\fpl_mag_ref';
    fmask_name = '\\cayyc-proj01\compute02\dchurikov\Tasks\MFP_interpolation\Fingerprints\Schnucks\Maps\fpl_mag_ref.mfp3';
end

% % Lowes_?entral_?harlotte_store
if 0
    %venue_id = 'Schnucks';
    venue_id = 'Lowes_?entral_?harlotte_store';

    x_sz = 254;
    y_sz = 179;
    cell_sz = 2;
    floor_number = 1;
    fname = '\\cayyc-proj01\compute02\dchurikov\Tasks\MFP_interpolation\Fingerprints\Lowes_Central_Charlotte_Store\maps\Lowes_Central_Charlotte_store';
    fmask_name = '\\cayyc-proj01\compute02\dchurikov\Tasks\MFP_interpolation\Fingerprints\Lowes_Central_Charlotte_Store\maps\Lowes_Central_Charlotte_store.mfp3';
end


% %ica_2p
% x_sz_in = 124;
% y_sz_in = 82;
% cell_sz_in = 2;
% cell_sz_out = 1;
% fname = 'c:\Temp\Cambrian\2m_thinned\fpl_mag';

%load source fingerprint
fname_in = strcat(fname,'.mfp3');
%[mag, mag_sigma] = LoadMfp3_ex (fname_in, x_sz_in, y_sz_in, cell_sz_in, floor_number);
[mag, mag_sigma] = LoadMfp3_mf (fname_in, x_sz, y_sz, cell_sz, floor_number);
fname_out = strcat(fname,'_int.mfp3');
PlotMfp(mag,strcat(venue_id, ', source MFP'));
PlotMfp(mag_sigma,strcat(venue_id, ', source MFP sigma'));

%load FP mask map
[mag_mask, mag_mask_sigma] = LoadMfp3_mf (fmask_name, x_sz, y_sz, cell_sz, floor_number);

fp_mask = CreateMask(mag_mask_sigma, venue_id);
PlotMfpMask(fp_mask, strcat(venue_id, ', MFP mask'));

[mag_ip, mag_sigma_ip] = InterpolateMfp(mag, mag_sigma, cell_sz, cell_sz, interpolation_method);

[mag_ip, mag_sigma_ip] = ApplyMask(mag_ip, mag_sigma_ip, fp_mask);

PlotMfp(mag_ip,strcat(venue_id, ', interpolated MFP'));
PlotMfp(mag_sigma_ip,strcat(venue_id, ', interpolated MFP sigma'));

save_mfp3(fname_out, mag_ip, mag_sigma_ip);



function [ mfp_ip, mfp_sigma_ip] = InterpolateMfp(mfp, mfp_sigma, cell_sz_in, cell_sz_out, interpolation_method)
    
    sz = size(mfp);
    if size(sz,2) == 3
        [mfp_ip, mfp_sigma_ip] = InterpolateMfp_SingleFloor (mfp, mfp_sigma, cell_sz_in, cell_sz_out, interpolation_method);
    elseif size(sz,2) == 4
        for i = sz(1):-1:1
            mfp_single_floor = reshape(mfp(i,:,:,:),sz(2)*sz(3)*sz(4),1);
            mfp_single_floor = reshape(mfp_single_floor,sz(2),sz(3),sz(4));
            mfp_sigm_single_floor = reshape(mfp_sigma(i,:,:,:),sz(2)*sz(3)*sz(4),1);
            mfp_sigm_single_floor = reshape(mfp_sigm_single_floor,sz(2),sz(3),sz(4));
            [mfp_ip(i,:,:,:), mfp_sigma_ip(i,:,:,:)] = InterpolateMfp_SingleFloor (mfp_single_floor, mfp_sigm_single_floor, cell_sz_in, cell_sz_out, interpolation_method);
        end
    else
        error('InterpolateMfp: wrong mfp');
    end
end

function [ mfp_ip, mfp_sigma_ip] = InterpolateMfp_SingleFloor(mfp, mfp_sigma, cell_sz_in, cell_sz_out, interpolation_method)
    %create mersh grid
    idx = 1;
    %xc = cell_sz_in/2 : cell_sz_in : x_sz_in;
    %yc = cell_sz_in/2 : cell_sz_in : y_sz_in;
    xc = cell_sz_in *(1:size(mfp,2)) - cell_sz_in/2;
    yc = cell_sz_in *(1:size(mfp,1)) - cell_sz_in/2;

    x = []; y = [];
    for i = 1:size(yc,2)
        for j = 1:size(xc,2)
            if((mfp_sigma(i,j,1) < 60) && (mfp_sigma(i,j,2) < 60)&& (mfp_sigma(i,j,3) < 60))
                x(idx) = xc(j);
                y(idx) = yc(i);
                mag_x(idx) = mfp(i,j,1);
                mag_y(idx) = mfp(i,j,2);
                mag_z(idx) = mfp(i,j,3);
                mag_sx(idx) = mfp_sigma(i,j,1);
                mag_sy(idx) = mfp_sigma(i,j,2);
                mag_sz(idx) = mfp_sigma(i,j,3);
                idx = idx+1;
            end
        end
    end

    
        %interpolation
        [Xq,Yq] = meshgrid(cell_sz_out *(1:size(mfp,2)) - cell_sz_out/2, ...
                           cell_sz_out *(1:size(mfp,1)) - cell_sz_out/2);                  

        xc_ip = cell_sz_out *(1:size(mfp,2)) - cell_sz_out/2;
        yc_ip = cell_sz_out *(1:size(mfp,1)) - cell_sz_out/2;                       
                       
        mfp_ip(size(Xq,1), size(Yq,2), 3) = 0;
        mfp_sigma_ip = 60*ones(size(yc_ip,2), size(xc_ip,2), 3);
                       
    if size(x,1) > 0
        F_mag_x = scatteredInterpolant(x',y',mag_x', interpolation_method);
        F_mag_y = scatteredInterpolant(x',y',mag_y', interpolation_method);
        F_mag_z = scatteredInterpolant(x',y',mag_z', interpolation_method);
        F_mag_sx = scatteredInterpolant(x',y',mag_sx', interpolation_method);
        F_mag_sy = scatteredInterpolant(x',y',mag_sy', interpolation_method);
        F_mag_sz = scatteredInterpolant(x',y',mag_sz', interpolation_method);

        mag_x_ip = F_mag_x(Xq,Yq);
        mag_y_ip = F_mag_y(Xq,Yq);
        mag_z_ip = F_mag_z(Xq,Yq);
        mag_sx_ip = F_mag_sx(Xq,Yq);
        mag_sy_ip = F_mag_sy(Xq,Yq);
        mag_sz_ip = F_mag_sz(Xq,Yq);

        %copy interpolated data to map        
        for i = 1:size(yc_ip,2)
            for j = 1:size(xc_ip,2)
                mfp_ip(i,j,1) = mag_x_ip(i,j);
                mfp_ip(i,j,2) = mag_y_ip(i,j);
                mfp_ip(i,j,3) = mag_z_ip(i,j);
                mfp_sigma_ip(i,j,1) = mag_sx_ip(i,j);
                mfp_sigma_ip(i,j,2) = mag_sy_ip(i,j);
                mfp_sigma_ip(i,j,3) = mag_sz_ip(i,j);
            end
        end
    end
end

function [mfp_out, mfp_sigma_out] = ApplyMask(mfp, mfp_sigma, fp_mask)
    sz = size(mfp);
    if size(sz,2) == 3
        [mfp_out, mfp_sigma_out] = ApplyMask_SingleFloor (mfp, mfp_sigma, fp_mask);
    elseif size(sz,2) == 4
        for i = sz(1):-1:1
            mfp_single_floor = reshape(mfp(i,:,:,:),sz(2)*sz(3)*sz(4),1);
            mfp_single_floor = reshape(mfp_single_floor,sz(2),sz(3),sz(4));
            mfp_sigm_single_floor = reshape(mfp_sigma(i,:,:,:),sz(2)*sz(3)*sz(4),1);
            mfp_sigm_single_floor = reshape(mfp_sigm_single_floor,sz(2),sz(3),sz(4));
            fp_mask_single_floor = reshape(fp_mask(i,:,:),sz(2)*sz(3),1);
            fp_mask_single_floor = reshape(fp_mask_single_floor,sz(2),sz(3));            
            [mfp_out(i,:,:,:), mfp_sigma_out(i,:,:,:)] = ApplyMask_SingleFloor (mfp_single_floor, mfp_sigm_single_floor, fp_mask_single_floor);
        end
    else
        error('InterpolateMfp: wrong mfp');
    end
end

function [mfp_ip, mfp_sigma_ip] = ApplyMask_SingleFloor(mfp, mfp_sigma, fp_mask)
    for i = 1:size(fp_mask,1)
        for j = 1:size(fp_mask,2)
            %if(is_in_mapped_area(xc_ip(j),yc_ip(i), x, y, cell_sz_in))
            if ((fp_mask(i, j) >= 0.5) && (fp_mask(i, j) < 1))
                mfp_ip(i,j,1) = mfp(i,j,1); % mfp - interpolated map
                mfp_ip(i,j,2) = mfp(i,j,2);  % case for interpolated areas (shadow)
                mfp_ip(i,j,3) = mfp(i,j,3);
                mfp_sigma_ip(i,j,1) = 20; %2*mfp_sigma(i,j,1);  % mfp_sigma - interpolated sigma
                mfp_sigma_ip(i,j,2) = 20; %2*mfp_sigma(i,j,2);
                mfp_sigma_ip(i,j,3) = 20; %2*mfp_sigma(i,j,3);
            elseif ((fp_mask(i, j) >= 0) && (fp_mask(i, j) < 0.5))
                mfp_ip(i,j,1) = 0;      % case for unavailable areas
                mfp_ip(i,j,2) = 0;
                mfp_ip(i,j,3) = 0;
                mfp_sigma_ip(i,j,1) = 60;
                mfp_sigma_ip(i,j,2) = 60;
                mfp_sigma_ip(i,j,3) = 60;
            else
                mfp_ip(i,j,1) = mfp(i,j,1); % case for previously mapped areas
                mfp_ip(i,j,2) = mfp(i,j,2);
                mfp_ip(i,j,3) = mfp(i,j,3);
                mfp_sigma_ip(i,j,1) = mfp_sigma(i,j,1);
                mfp_sigma_ip(i,j,2) = mfp_sigma(i,j,2);
                mfp_sigma_ip(i,j,3) = mfp_sigma(i,j,3);
            end
        end
    end
end

function res = is_in_mapped_area(xc_ip,yc_ip, x, y, cell_size)
    res = 0;
    for i = 1 : size(x,2)
        if ( (abs(xc_ip - x(i)) <= cell_size / 2) && ...
             (abs(yc_ip - y(i)) <= cell_size / 2))
            res = 1;
            break;
        end
    end
end

function fp_mask = CreateMask (mag_mask_sigma, venue_id)

    sz = size(mag_mask_sigma);
    if size(sz,2) == 3
        fp_mask = CreateMask_singlefloor (mag_mask_sigma, venue_id, 1);
    elseif size(sz,2) == 4
        for i = 1:sz(1)
            mask_single_floor = mag_mask_sigma(i,:,:,:);
            mask_single_floor = reshape(mask_single_floor,sz(2)*sz(3)*sz(4),1);
            mask_single_floor = reshape(mask_single_floor,sz(2),sz(3),sz(4));
            fp_mask(i,:,:,:) = CreateMask_singlefloor (mask_single_floor, venue_id, i);
        end
    else
        fp_mask = [];
    end
end

function fp_mask = CreateMask_singlefloor (inp_mag_mask, venue_id, floor_id)
    
    fp_mask(size(inp_mag_mask,1), size(inp_mag_mask,2)) = 0;

    switch venue_id
        case 'Northland'
            if (floor_id == 1)
                fp_mask = CreateFromTemplate(inp_mag_mask, fp_mask);
                fp_mask = AdjustFrameForNorthland(fp_mask);
                fp_mask = AdjustMaskForNorthland(fp_mask);
            end
        case 'Northland shadow'
             if (floor_id == 1)
                fp_mask = CreateFromTemplate(inp_mag_mask, fp_mask);
                fp_mask = AdjustFrameForNorthland(fp_mask);
                fp_mask = AddShadow(fp_mask);
             end
        case 'Schnucks'
            fp_mask = CreateFromTemplate(inp_mag_mask, fp_mask);
            fp_mask = AdjustMaskForSchnucks(fp_mask);            
        case 'Schnucks shadow'
            fp_mask = CreateFromTemplate(inp_mag_mask, fp_mask);
            fp_mask = AddShadow(fp_mask);
        otherwise
            fp_mask = CreateFromTemplate(inp_mag_mask, fp_mask);
            fp_mask = AddShadow(fp_mask);
    end
end

function fp_mask = CreateFromTemplate(fp_template, fp_mask_in)
    fp_mask = fp_mask_in;
    for i = 1:size(fp_mask_in,1)
        for j = 1:size(fp_mask_in,2)
            if(fp_template(i,j,1) < 60)
                fp_mask(i,j) = 1;
            end
        end
    end
end

function fp_mask = AddShadow(fp_mask_in)
    shadow_size = 1;
    shadow_scale = 0.7;
    sz = size(fp_mask_in);    
    fp_mask = fp_mask_in;%fp_mask(sz(1), sz(2)) = 0;
    for i = 1:sz(1)
        for j = 1:sz(2)
            %if(fp_mask_in(i,j) > 0) % multy 'gray' shadow case
            if(fp_mask_in(i,j) == 1)
                fp_mask(i,j) = fp_mask_in(i,j);
                for ii = i-shadow_size : i+shadow_size
                    for jj = j-shadow_size : j+shadow_size
                        if ( (ii > 0)&& (jj > 0) &&(ii <= sz(1))&& (jj <= sz(2)))
                            if (fp_mask(ii, jj) ~= 1) 
                                fp_mask(ii, jj) = shadow_scale*fp_mask(i,j);
                            end
                        end
                    end
                end
            end
        end
    end
end

function fp_mask = AdjustFrameForNorthland(fp_mask0)

    fp_mask = fp_mask0;

    fp_mask(62:70,80:100) = 0;
    fp_mask(1:6,1:200) = 0;
    fp_mask(59:65,170:180) = 0;
    fp_mask(50:70,175:190) = 0;
    
    end

function fp_mask = AdjustMaskForNorthland(fp_mask0)

    fp_mask = fp_mask0;

    fp_mask(22:47,75:165) = 0.5;
    fp_mask(27:38,63:75) = 0.5;
    fp_mask(20:30,165:185) = 0.5;
    fp_mask(5:23,172:183) = 0.5;
    fp_mask(5:23,81:90) = 0.5;
    
    for i = 1:size(fp_mask0,1)
        for j = 1:size(fp_mask0,2)
            if(fp_mask0(i,j) > 0)
                fp_mask(i,j) = fp_mask0(i,j);
            end
        end
    end
    

    fp_mask(52:59,77:87) = 1;
    fp_mask(51:51,79:86) = 1;
    fp_mask(50:50,80:86) = 1;
    fp_mask(49:49,81:86) = 1;

    fp_mask(40:47,82:86) = 1;
    fp_mask(41:46,88:89) = 1;
    fp_mask(42:44,91:161) = 1;

    fp_mask(30:34,65:87) = 1;
    fp_mask(35:38,81:87) = 1;

    fp_mask(27:29,80:89) = 1;

    fp_mask(24:26,82:161) = 1;
    fp_mask(33:34,161) = 1;

    fp_mask(27,163) = 1;

    fp_mask(8:22,175:179) = 1;

    fp_mask(8:24,84:86) = 1;
    fp_mask(54:55,170:171) = 1;
    
    
end

function fp_mask = AdjustMaskForSchnucks(fp_mask0)

    fp_mask = fp_mask0;

    fp_mask(23:77,76:118) = 0.5;
    fp_mask(24:80,52:76) = 0.5;
    fp_mask(25:74,34:52) = 0.5;
       
    for i = 1:size(fp_mask0,1)
        for j = 1:size(fp_mask0,2)
            if(fp_mask0(i,j) > 0)
                fp_mask(i,j) = fp_mask0(i,j);
            end
        end
    end
end

function PlotMfp (mfp , mfp_title)
    sz = size(mfp);
    if size(sz,2) == 3
        PlotMfpFloor(mfp, mfp_title);
    elseif size(sz,2) == 4
        for i = 1:sz(1)
            mfp_floor = mfp(i,:,:,:);
            mfp_floor = reshape(mfp_floor,sz(2)*sz(3)*sz(4),1);
            mfp_floor = reshape(mfp_floor,sz(2),sz(3),sz(4));
            PlotMfpFloor (mfp_floor, strcat( mfp_title, ', floor ', int2str(i)));
        end
    else
        error('PlotMfp error: incorrect MFP format');
        
    end
end

function PlotMfpFloor (mfp_floor, mfp_title)
    figure;
    subplot(3,1,1);pcolor(mfp_floor(:,:,1)); axis equal;    title(strcat( mfp_title, ', mx'));
    subplot(3,1,2);pcolor(mfp_floor(:,:,2)); axis equal;    title(strcat( mfp_title, ', my'));
    subplot(3,1,3);pcolor(mfp_floor(:,:,3)); axis equal;    title(strcat( mfp_title, ', mz'));
end

function PlotMfpMask (mfp_mask , mask_title)
    sz = size(mfp_mask);
    if size(sz,2) == 2
        PlotMfpFloor(mfp_mask, mask_title);
    elseif size(sz,2) == 3
        for i = 1:sz(1)
            floor_mask = mfp_mask(i,:,:,:);
            floor_mask = reshape(floor_mask,sz(2)*sz(3),1);
            floor_mask = reshape(floor_mask,sz(2),sz(3));
            PlotFloorMask (floor_mask, strcat( mask_title, ', floor ', int2str(i)));
        end
    else
        error('PlotMfpMask error: wrong MFP mask');
        
    end
end

function PlotFloorMask (mfp_floor, mfp_title)
    figure;
    pcolor(mfp_floor(:,:,1)); axis equal;    title(mfp_title);
end