import io
import csv
import Geo2LocalCoordinateConverter

# reading the position file        
def reading_positions_file(positions_file):
    f = open(positions)
    p = [0, 0, 0, 0]
    i = 2
    for line in f:
        reader = csv.reader(io.StringIO(line), delimiter=' ')
        for solv in reader:
            for n in solv:
                if(len(n) > 0):
                    p[i] = n
                    i += 1
                    #print (n, "    ", end='')
                    if(i > 3):
                        i = 2
                        break
            #print("")
            one_time_particles.append(p)
            one_time_geopositions = Geo2LocalCoordinateConverter.Local2GeoAllParticles(one_time_particles)
            #print(one_time_geopositions)
            geopositions.append(one_time_geopositions)
            one_time_particles = []
    return geopositions

