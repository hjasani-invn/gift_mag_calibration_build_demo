function [mag_mf, mag_sigma_mf] = LoadMfp3_mf(fName, mfSizeX, mfSizeY, mfCellSize, floorCount)

for floor = 1 : floorCount
        [mag, mag_sigma] = LoadMfp3_ex(fName, mfSizeX, mfSizeY, mfCellSize, floor);
        mag_mf(floor,:,:,:) = mag;
        mag_sigma_mf(floor,:,:,:) = mag_sigma;
    end

end