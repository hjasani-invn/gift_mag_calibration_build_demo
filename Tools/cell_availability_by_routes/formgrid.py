import math
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import color_map

def formgrid(points_list, max_x, max_y, cellsize, out_file) :
	#max_index_x = math.trunc(max_x / cellsize + 1)
	#max_index_y = math.trunc(max_y / cellsize + 1)
	max_index_x = math.trunc(max_x / cellsize )
	max_index_y = math.trunc(max_y / cellsize )
	max_index_sum = max_index_x * max_index_y
	print("number of ceils for x = ", max_index_x, "number of ceils for y = ", max_index_y, "common number of ceils = ", max_index_sum)
	table = {}
	for point in points_list:
		# get index of cell
		index_x  = math.trunc(point[0] / cellsize )
		index_y  = math.trunc(point[1] / cellsize )
		index_sum = index_y * max_index_x + index_x
		#print(point[0], "     ", point[1], "     ", index_x, "     ", index_y, "     ", index_sum )
		table[index_sum] = 1


	table_file = open(out_file, "wb")
	#index_sum = 0
	availability = np.zeros([max_index_x, max_index_y])
	#while index_sum < max_index_sum :
	for i in range(0, max_index_x):
		for j in range(0, max_index_y):
			index_sum = j * max_index_x + i
			#index_sum = i * max_index_y + j

			value = table.get(index_sum)
			if value != None :
				res = bytes ([ 1, 0 ])
				availability[i,j] = 4.5
			else:
				res = bytes ([ 0, 0 ])
			table_file.write(res)
			#table_file.write((''.join(chr(res))).encode('ascii')) 
			#print(index_sum, "      ", value)
			#index_sum += 1

	table_file.close()

	A = color_map.get_colormap()
	the_colormap = mpl.colors.ListedColormap(A / 255.0)

	#outpath = os.path.join(output_directory, availability_plot_file)
	plt.subplot(2, 2, 1)
	plt.pcolor(availability.transpose(), cmap=the_colormap, vmin=0, vmax=5, edgecolors='k', linewidths=0.3)
	#plt.pcolor(availability, cmap=the_colormap, vmin=0, vmax=5, edgecolors='k', linewidths=0.3)
	#plt.xlabel("xlabel")
	#plt.ylabel("ylabel")
	#plt.axis("on")
	#plt.colorbar()
	#plt.axis([0, max_index_x, 0, max_index_y])
	plt.subplots_adjust(bottom=0)
	plt.subplots_adjust(top=1)
	plt.subplots_adjust(right=1)
	plt.subplots_adjust(left=0)
	plt.savefig("availability.png", pad_inches=0, dpi=200)
	plt.close()

	return table
	