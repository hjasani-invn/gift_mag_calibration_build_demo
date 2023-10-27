% initialization
fName = '\\cayyc-proj01\compute02\vpentyukhov\fingerprints\ICA_crowdsource\reference\ICA_CrowdSource.mfp3';
%fName = 'c:\Users\vpentyukhov\Development\InvenSenseInc\Gift\Tools\mfp_map_update\ICA_Survey+CrowdSource.mfp3';
fName_crowdsource = '\\cayyc-proj01\compute02\vpentyukhov\fingerprints\ICA_crowdsource\Crowdsource_results\output_May_2020_tests_current_results_no_calibr_flag\ICA_CrowdSource.mfp3';

mfSizeX = 77.285;
mfSizeY = 101.63;
mfCellSize = 2;

% downloading fprints
floorNumber = 6;
[map, mag_sigma] = LoadMfp3_ex(fName, mfSizeX, mfSizeY, mfCellSize, floorNumber);

floorNumber = 6;
[map_crowdsource, mag_sigma_crowdsource] = LoadMfp3_ex(fName_crowdsource, mfSizeX, mfSizeY, mfCellSize, floorNumber);


% calculation
map_cs_notzero1 = find(map_crowdsource ~= 0);
map_cs_notzero = [];
map_notzero = find(map ~= 0);
for n = 1 : length(map_notzero)
    s = find(map_cs_notzero1 == map_notzero(n));
    if (length(s) > 0)
        %fprintf('%10d', map_cs_notzero1(s));
        map_cs_notzero = [map_cs_notzero map_cs_notzero1(s)];
    end
end

delta = map(map_cs_notzero) - map_crowdsource(map_cs_notzero);

% output
numX = ceil(mfSizeX/mfCellSize);
numY = ceil(mfSizeY/mfCellSize);
lengthOne = numX*numY;

x = find (map_cs_notzero <= lengthOne);
y = find ((map_cs_notzero > lengthOne) & map_cs_notzero <= 2*lengthOne);
z = find ((map_cs_notzero > 2*lengthOne));

%fprintf('x coordinate\n')
for n = 1 : length(x)
    jj = ceil(map_cs_notzero(n)/numY);
    ii = map_cs_notzero(n) - (jj-1) *numY;
    %fprintf('%10d %10d %10d', map_cs_notzero(n), ii, jj );
    %fprintf('%10d %10d', ii, jj );
    fprintf('%10d ,', 1 );
    fprintf('%15f , %15f ,', (jj-1)*mfCellSize+mfCellSize/2, (ii-1)*mfCellSize+mfCellSize/2 );
    %fprintf('%15f\n', map_crowdsource(map_cs_notzero(n)));
    fprintf('%15f\n', delta(n));
end
fprintf(',\n');

%fprintf('y coordinate\n')
for n = 1 : length(y)
    jj = ceil( (map_cs_notzero(length(x) + n)-lengthOne)/numY);
    ii = map_cs_notzero(length(x) + n)-lengthOne - (jj-1) *numY;
    %fprintf('%10d %10d %10d', map_cs_notzero(length(x) + n), ii, jj );
    %fprintf('%10d %10d', ii, jj );
    fprintf('%10d ,', 2 );
    fprintf('%15f , %15f ,', (jj-1)*mfCellSize+mfCellSize/2, (ii-1)*mfCellSize+mfCellSize/2 );
    %fprintf('%15f\n', map_crowdsource(map_cs_notzero(length(x) + n)));
    fprintf('%15f\n', delta(length(x) + n));
end
fprintf(',\n');

%fprintf('z coordinate\n')
for n = 1 : length(z)
    jj = ceil( (map_cs_notzero(length(x) + length(y) + n)-2*lengthOne)/numY);
    ii = map_cs_notzero(length(x) + length(y) + n)-2*lengthOne - (jj-1) *numY;
    %fprintf('%10d %10d %10d', map_cs_notzero(length(x) + length(y) + n), ii, jj );
    %fprintf('%10d %10d', ii, jj );
    fprintf('%10d ,', 3 );
    fprintf('%15f , %15f ,', (jj-1)*mfCellSize+mfCellSize/2, (ii-1)*mfCellSize+mfCellSize/2 );
    %fprintf('%15f\n', map_crowdsource(map_cs_notzero(length(x) + length(y) + n)));
    fprintf('%15f\n', delta(length(x) + length(y) + n));
end



