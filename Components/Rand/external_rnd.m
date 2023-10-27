% Uniformity random number generator [0 32767]
function [rand, next_state] = external_rnd(current_state)
    state = bitand(uint64( uint64(fix(current_state)) * 1103515245 + 12345), uint64(4294967295) );
    next_state = uint64(state);
    tmp = bitshift(next_state, -16);
    rand64 = bitand(  tmp  , uint64(32767) );
    rand = uint32(rand64);
end
