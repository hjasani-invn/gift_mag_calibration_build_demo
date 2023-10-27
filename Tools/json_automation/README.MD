The tool is intended to create json file from venue tempale and specific venue parameters

python version: 3.6 or late

main file: ja_main.py

command line: python ja_main.py <venue_template.json> <venue_specifics.json> <coordinates.txt> <floors.json>

'venue_template.json'
This is a template with general settings actual for most venues. The file is provided as part of tool.

'venue_specifics.json'
This is a json file with specific settings for the venue. The file should be provided by a person or a tool which initiate a venue.json generation.
This json file must contains the following settings:
{
  "name": "venue_name", // venue name
  "venue" :
        {
            "id" : 55,      // venue id
            "magX": nn.nn,  // [uT] > mean expected Earth magnetic field in the venue
            "magY": nn.nn,  // [uT] > can be obtained from: 
            "magZ": nn.nn   // [uT] > https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfwmm
        },
  "magnetic_enable": true,       // [true is recommended] eneble/disable magnetic fingerprint generation
  "WiFi_enable" : true,          // [true is recommended] eneble/disable WiFi fingerprint generation
  "BLE_enable" : false,          // [false is recommended] eneble/disable BLE fingerprint generation
  "ble_proximity_enable" : true  // [true is recommended] eneble/disable proximity DB generation
}
The file can contain any other settings. Settings from this file overwrite same settings from venue_template.json.
See description of all available settings in description_of_venue.json
  
'coordinates.txt'
This is a file with coordinates of venue corners and venue dimensions. The file is provided by a person or a tool which provides venue overlays.
For multi-floor venue this file must consist coordinates of rectangular footprint on Eath surface which covers overlays of all floor.

'floors.json' 
This is a file with floors description. The file is provided by a person or a tool that provides floor parameters.

output: venue_name.json  - generated json file with venue parameters

Examples of input/output files are in examples' folder

Example of usage:
python ja_main.py template.json venue.json coordinates.txt floors.json
