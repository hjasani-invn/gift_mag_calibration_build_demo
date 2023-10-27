% normal random number generator mean == 0 and dispersion
function [norm_rand, next_state] = external_rnd_norm(current_state)
   sum = 0;
   N = 12;
   next_state = current_state;
   for i = 1:N
       [rand, next_state] = external_rnd(next_state); 
       sum = sum + double(rand);
   end
   norm_rand = ( sum * ( 1. / 32767  ) - N / 2 );
end
