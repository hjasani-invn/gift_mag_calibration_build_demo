% A is calculated track with x,y,timestamp
% B is reference track with x,y,timestamp
function  avererr = calc_diff(A, B, savefile) 

fout = fopen(savefile, 'wt');

lengthA = length(A(:,1));
lengthB = length(B(:,1));

nearestindex = 1;
count = 0;

fprintf(fout, 'calc log timestamp   x1       y1       reference log timestamp   x2    y2     error( sqrt( (x1-x2)^2 + (y1-y2)^2 ))\n');

for i=1:lengthA
      curtime = A(i,3);
      nearestflag = 0;
      for j=nearestindex:lengthB-1
          if (curtime >= B(j,3)) && (curtime < B(j+1,3))
             if (curtime - B(j,3)) <= (B(j+1,3) - curtime)
                nearestindex = j;
             end    
             if (curtime - B(j,3)) > (B(j+1,3) - curtime)
              nearestindex = j+1;             
             end 
             nearestflag = 1;
             break
          end           
      end
      if nearestflag == 1;
          err = sqrt( (A(i,1)-B(nearestindex,1))^2 + (A(i,2)-B(nearestindex,2))^2 );
          fprintf(fout, '%.5f  %.5f  %.5f  %.5f  %.5f  %.5f  %.5f\n', A(i,3), A(i,1), A(i,2), B(nearestindex,3) , B(nearestindex,1) , B(nearestindex,2), err);
          count = count + 1;
          Er(count) = err;
      end
end

    avererr = mean(Er);
    fprintf(fout, '\naverage error = %.5f\n', avererr);
    fclose(fout);
end
