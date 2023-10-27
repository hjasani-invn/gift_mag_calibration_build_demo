import io
import csv
import Geo2LocalCoordinateConverter

# reading the particles file
def reading_particles_file(particles_file):
    count = 0
    timestamp = 0
    one_time_particles = []
    geopositions = []
    time_count = 0
    f = open(particles_file)
    for line in f:
        reader = csv.reader(io.StringIO(line), delimiter=';')
        for particle in reader:
            if len(particle) == 6:
                particle.append(0)
            if (count == 0):
                one_time_particles.append(particle)
            else: #(count > 0)
                if(timestamp == particle[1]):
                    one_time_particles.append(particle)
                else:
                    time_count += 1
                    if(time_count == 10):
                        print("time = ", float(particle[1]) / 1000)
                        time_count = 0

                    # geographical coordinates (lat, lon) array
                    one_time_geopositions = Geo2LocalCoordinateConverter.Local2GeoAllParticles(one_time_particles)
                    geopositions.append(one_time_geopositions)
                    count = 0
                    one_time_particles = []
                    one_time_particles.append(particle)
            count += 1
            timestamp = particle[1]
    return geopositions
