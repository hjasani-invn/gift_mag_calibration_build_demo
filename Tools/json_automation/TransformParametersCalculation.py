#TransformParametersCalculation function calculates parameters of transformation 
#to set bind beetwen geographic coordinate of particle and
#its position on the screen
import math
import numpy as np

M_PI = (3.1415926535897932384626433832795)

def TransformParametersCalculation(geo, local):

    geo[0,0] = geo[0,0] /180*M_PI
    geo[0,1] = geo[0,1] /180*M_PI
    geo[1,0] = geo[1,0] /180*M_PI
    geo[1,1] = geo[1,1] /180*M_PI
    geo[2,0] = geo[2,0] /180*M_PI
    geo[2,1] = geo[2,1] /180*M_PI
    geo[3,0] = geo[3,0] /180*M_PI
    geo[3,1] = geo[3,1] /180*M_PI
    #print("geo",geo)

    h_lat = np.zeros((4, 5))
    h_lon = np.zeros((4, 5))
    lat = np.zeros(4)
    lon = np.zeros(4)

    AttParams = np.zeros(5)

    pi = M_PI

    #      1    2       3          4      5
    #x = [lat0, lon0, alfa_lat, alfa_lon, psy]
    #X = [0, 0, 2, 5, -pi/2];
    X = np.zeros((1, 5))
    X[0,0] = 0
    X[0,1] = 0
    X[0,2] = 1
    X[0,3] = 1
    X[0,4] = -pi/2
    #X(1) = mean(geo(:,1)); from matlab
    #X(2) = mean(geo(:,2)); from matlab
    X[0,0] = geo[:,0].mean()
    X[0,1] = geo[:,1].mean()
  
    dgeo_1 = geo[0,0] - geo[1,0]
    dgeo_2 = geo[0,1] - geo[1,1]
    dgeo = math.sqrt(dgeo_1*dgeo_1 + dgeo_2*dgeo_2)
    dlocal_1 = local[0,0] - local[1,0]
    dlocal_2 = local[0,1] - local[1,1]
    dlocal = math.sqrt(dlocal_1*dlocal_1 + dlocal_2*dlocal_2)

    X[0,2] = dgeo/dlocal;
    X[0,3] = dgeo/dlocal;
  
    dx =np.array([1,1,1,1,1])
    norm_dx = np.linalg.norm(dx)
    
    while (norm_dx > 1e-7):
    
     #H matrix calculation
     sz = np.size(geo,0)

     #for (i = 1:sz) from matlab
     for i in range(sz):

       h_lat[i,0] = 1
       h_lat[i,1] = 0
       h_lat[i,2] = local[i,0]*math.cos(X[0,4]) - local[i,1]*math.sin(X[0,4])
       h_lat[i,3] = 0
       h_lat[i,4] = -local[i,0]*X[0,2]*math.sin(X[0,4]) - local[i,1]*X[0,2]*math.cos(X[0,4])
  
       h_lon[i,0] = 0
       h_lon[i,1] = 1
       h_lon[i,2] = 0
       h_lon[i,3] = local[i,0]*math.sin(X[0,4]) + local[i,1]*math.cos(X[0,4]);
       h_lon[i,4] = -local[i,0]*X[0,3]*math.sin(X[0,4]) - local[i,1]*X[0,3]*math.cos(X[0,4])
  
       lat[i] = X[0,0] + local[i,0]*X[0,2]*math.cos(X[0,4]) - local[i,1]*X[0,2]*math.sin(X[0,4])
       lon[i] = X[0,1] + local[i,0]*X[0,3]*math.sin(X[0,4]) + local[i,1]*X[0,3]*math.cos(X[0,4])
     
     #H = [h_lat; h_lon]; from matlab
     ##print("h_lat = ", h_lat)
     #print("h_lon = ", h_lon)
     H = np.append(h_lat, h_lon, axis=0)
     #print("H =", H)
     #y = [lat'; lon'];  from matlab
     #print("lat = ", lat)
     #print("lon = ", lon)
     y1 = np.append(lat, lon, axis=0)
     #print("y1 =", y1)
     y2 = y1.reshape(1, len(y1))
     #print("y2 =", y2)
     y = y2.T
     #print("y =", y)
     #z = [geo(:,1); geo(:,2)];  from matlab
     #print("geo[:,0] = ", geo[:,0])
     #print("geo[:,1] = ", geo[:,1])
     z1 = np.append(geo[:,0], geo[:,1], axis=0)
     #print("z1 =", z1)
     z2 = z1.reshape(1, len(z1))
     #print("z2 =", z2)
     z = z2.T
     #print("z =", z)
  
     dy = z-y;
     
     #if (rcond(H'*H) < 1e-23)  from matlab
     if (np.linalg.det( np.matmul(H.transpose(),H)) < 1e-23):
       X[0,4] = X[0,4] + pi/2;
       dx = np.array([0,0,0,0,0])
       norm_dx = 1;
     else:
       #K = (H'*H)^(-1)*H';  from matlab
       K = np.matmul( np.linalg.inv((np.matmul(H.transpose(),H))) , H.transpose() )
       #dx = K*dy;
       dx = np.matmul( K , dy)
       dx = dx * 0.5
       if (abs(dx[4]) > 2*pi):
         X[0,4] = X[0,4] + pi/2
         dx = np.array([0,0,0,0,0])
         norm_dx = 1
       else:
         norm_dx = np.linalg.norm(dx)
     
     if (X[0,4] > 7):
       print('Can''t calculate transformation parameters by specified data set')
     #X = X + dx';  from matlab
     X = X + dx.transpose();
    
    AttParams[0] = X[0,0];
    AttParams[1] = X[0,1];
    if ((X[0,2]> 0) and (X[0,3]>0)):
      AttParams[2] = X[0,2];
      AttParams[3] = X[0,3];
      AttParams[4] = X[0,4];
    elif ( (X[0,2] < 0 ) and  ( X[0,3] < 0 ) ):
      AttParams[2] = -X[0,2];
      AttParams[3] = -X[0,3];
      AttParams[4] = X[0,4] + -pi*np.sign(X[0,4]);    
    elif (X[0,2]< 0):
      AttParams[2] = -X[0,2];
      AttParams[3] = -X[0,3];
      AttParams[4] = X[0,4] + -pi*np.sign(X[0,4]);
    else:
      AttParams[2] = X[0,2];
      AttParams[3] = X[0,3];
      AttParams[4] = X[0,4];

    AttParams[0] = AttParams[0] *180/M_PI
    AttParams[1] = AttParams[1] *180/M_PI
    AttParams[4] = AttParams[4] *180/M_PI

    return AttParams

# for testing only 
if  __name__ == "__main__":

    #1  0    590       38.60185487701      -90.44140364513 
    #2  900  590        38.60187043109      -90.44304511914 
    #3  900  0          38.60102848888      -90.44305807485 
    #4  0    0          38.6010129348       -90.44141660085 

    a = np.array([[1, 2, 3], [4, 5, 6]], float)


    geo = np.array([ [38.60185487701 /180*M_PI   ,    -90.44140364513 /180*M_PI ], [38.60187043109 /180*M_PI  ,  -90.44304511914 /180*M_PI], [38.60102848888 /180*M_PI   ,   -90.44305807485 /180*M_PI ] , [38.6010129348 /180*M_PI   ,    -90.44141660085 /180*M_PI ] ], float)
    local = np.array([ [ 0  ,  590 ], [900 , 590 ] , [900 , 0] , [0  ,  0] ], float)
   
    AttParams = TransformParametersCalculation(geo, local)

    #reference
    #lat0=38.601012934800004
    #lon0=-90.441416639350720
    #alfa_lat=0.000000024908036
    #alfa_lon=0.000000031834685
    #heading=-89.306138247561861

    #result
    print("lat0 = ", AttParams[0])
    print("lon0 = ",AttParams[1])
    print("alfa_lat = ",AttParams[2])
    print("alfa_lon = ",AttParams[3])
    print("heading = ",AttParams[4])
