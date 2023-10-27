#!/bin/sh
#
#./fpbuilder.out --settings /projects/compute02/dchurikov/Polygon/fp_builder.console/settings/core_m1-default-lyn.json > /projects/compute02/dchurikov/Polygon/fp_builder.console/settings/core_m1-default-lyn.console.log 2>&1
./fpbuilder.out --settings ./../venues/ISJ/venue_linux.json  1> run.log 2> err.log
