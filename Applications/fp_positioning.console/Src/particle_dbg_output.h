
/*
Description of debug output for particles visualisation

Data are saved in csv-file, one line contains information about one particle or filter solution.
string format: time_tag,lat,lon,floor,w,h,type

The output is controled by json commands:
"mixed_particles_out" : 1/0 - enble/disable mixed filter particles and mixed solution in the debug file
"mag_particles_out" : 1/0 - enble/disable mag filter particles out mag solution in the debug file
There is no any debug output if json does not contain the settings specified above.
*/

/*
type - particle type, 0 - none/general, 1 - from mixed filter, 2 - from magnetic filter, 3 - from starting filter,
                      10 - mixed position, 11 - mag position, 12 - wifi position, 13 - ble posiytion, 14 - proximity position
*time_tag - time as uint64_t tag in miliseconds
x - local position in meters
y - local position in meters
floor - floor as double 
w - particle weigt as double
h - heading as double in degrees, reserved, default value = 0
/
