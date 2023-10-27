function [ roll, pitch, heading ] = TPN_dcm2angle( C )

    % between +/-pi
    roll = atan2( C(3, 2), C(3,3) );
    % between +/-pi/2
    pitch = atan( -C(3,1) / sqrt( C(3, 2)^2 + C(3, 3)^2 ) );
    
    % between +/-pi
    heading = atan2( C(2, 1), C(1, 1) );

end

