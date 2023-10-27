=========Using the scripts=============


Set "cellsize", "max_x" and "max_y" variables in the main function of the script (all in meters).
Set "floor_number" variable in the main function (starts from 0).
Set "file1" and "file2" variables as file paths for the two WFP databases in "wifi3" format (can be produced from "wifi4" format by deleting the header).

Input parameters:
	1) settings_json
	2) wfp_1 - 1st WFP file to compare
	3) wfp_2 - 2nd WFP file to compare
	4) floor_number
	


Output is:
	1) "image.png" is graphical representation of difference between cells in two WFPs.
		(difference is a square root of the sum iterating for each mac address in a given cell: 
			sqrt(sum_by_mac((m1(f1)*w1(f1) - m1(f2)*w1(f2) + m2(f1)*w2(f1) - m2(f2)*w2(f2))^2))
	
	2) "AP_metric.txt" - file with the list of mac addresses and their difference (zero if only one WFP contains this mac address)
		(difference for one mac address is the square root of the sum iterating for each cell on a given floor and for given mac address:
			sqrt(sum_by_cell((m1(f1)*w1(f1) - m1(f2)*w1(f2) + m2(f1)*w2(f1) - m2(f2)*w2(f2))^2))


=======================================


