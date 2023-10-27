import re
import os
import sys
sys.path.insert(0, '..')
import folder_parser
import matplotlib as mpl
import matplotlib.pyplot as plt
mpl.use('TkAgg')
#mpl.use('Agg')

if __name__ == "__main__":
	print("")
	scipt_path = os.path.dirname(sys.argv[0])
	full_scipt_path = os.path.abspath(scipt_path)

	root_path = sys.argv[1].replace("\r", "")
	os.chdir(root_path)
	dir_pattern = '.*'

	for cur, dirs, files in os.walk(root_path):
		for d in dirs:
			if re.match(dir_pattern, d) is not None:
				path, out_venue_folder = os.path.split(cur)

				input_file_list = folder_parser.plot_floor(os.path.join(cur, d), 'mixed_out_dbg.log')
				#input_file_list = folder_parser.plot_floor_float(os.path.join(cur, d), 'pf_log_init.log')
				#input_file_list = folder_parser.plot_converter_height(os.path.join(cur, d), '.*converter_c.txt')

				os.chdir(full_scipt_path)
				os.chdir(root_path)
				#print("======================")

	#plt.grid()
	#plt.show()
