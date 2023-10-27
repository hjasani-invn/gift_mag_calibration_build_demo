import re
import os
import sys
sys.path.insert(0, '..')
import folder_parser
import reader_blp
import logging

def write_model_output_file(input_file_folder, times_interp, geo_lat_interp, geo_lon_interp, floors_interp,
							coord_sigma_interp, stride_lengths):
	template = "{}, {}, {}, {:12.3f}, {:12.3f},    {},    "
	template_forstride_lengths = "  {:12.3f}"

	fname = os.path.join(input_file_folder, 'irl_data_mt_converter.txt')
	f = open(fname, 'w')

	for n in range(len(times_interp)):
		str = template.format(times_interp[n], geo_lat_interp[n], geo_lon_interp[n],
							  coord_sigma_interp[n], coord_sigma_interp[n], floors_interp[n])
		str += ' 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,' + \
			   template_forstride_lengths.format(stride_lengths[n]) + ', 0'
		#print(str)
		f.write(str + '\n')

def write_blp_output_file(input_file_folder, blp_data) :
	fname = os.path.join(input_file_folder, 'blp_debug_converter.log')
	f = open(fname, 'w')

	for str in blp_data:
		f.write(str + '\n')


def write_wifi_output_file(input_file_folder, wifi_data) :
	fname = os.path.join(input_file_folder, 'wifi_debug_converter.log')
	f = open(fname, 'w')

	for str in wifi_data:
		f.write(str + '\n')


mixed_out_dbg = '.*(mixed_out_dbg.log|fpp_mixed.txt)'
ble_proximity_log_init = '.*(ble_proximity_log_init.log|proximity_log_c.txt)'
wifi_log_init = '.*(wifi_log_init.log|wifi_log_c.txt)'
#mixed_out_dbg = 'fpp_mixed.txt'
#ble_proximity_log_init = 'ble_proximity_log_init.txt'
#wifi_log_init = 'wifi_log_init.txt'


if __name__ == "__main__":
	version = "1.0.0"
	try:
		print("Creator of model logs for tpn, version ", version)
		scipt_path = os.path.dirname(sys.argv[0])
		full_scipt_path = os.path.abspath(scipt_path)

		root_path = sys.argv[1].replace("\r", "")
		os.chdir(root_path)
		dir_pattern = '.*'
		if len(sys.argv) > 2:
			blp4_file = sys.argv[2].replace("\r", "")
			#ble_map = reader_blp3.read(blp3_file)
			ble_map = reader_blp.read_4format(blp4_file)
			uuid = ble_map[0][0]
		else:
			uuid = '00000000-0000-0000-0000-000000000000'

		for cur, dirs, files in os.walk(root_path):
			for d in dirs:
				if re.match(dir_pattern, d) is not None:
					path, out_venue_folder = os.path.split(cur)

					input_file_folder, times_interp, geo_lat_interp, geo_lon_interp, floors_interp, \
						coord_sigma_interp, stride_lengths = \
						folder_parser.interpolate(os.path.join(cur, d), mixed_out_dbg, 50)

					if len(input_file_folder) > 0:
						write_model_output_file(input_file_folder, times_interp, geo_lat_interp, geo_lon_interp,
												floors_interp,	coord_sigma_interp, stride_lengths)

					os.chdir(full_scipt_path)
					os.chdir(root_path)
					#print("======================")

		for cur, dirs, files in os.walk(root_path):
			for d in dirs:
				if re.match(dir_pattern, d) is not None:
					path, out_venue_folder = os.path.split(cur)

					input_file_folder, blp_data = \
						folder_parser.create_blp_log(os.path.join(cur, d), ble_proximity_log_init, uuid)

					if len(input_file_folder) > 0:
						write_blp_output_file(input_file_folder, blp_data)

					os.chdir(full_scipt_path)
					os.chdir(root_path)
				#print("======================")

		for cur, dirs, files in os.walk(root_path):
			for d in dirs:
				if re.match(dir_pattern, d) is not None:
					path, out_venue_folder = os.path.split(cur)

					input_file_folder, wifi_data = \
						folder_parser.create_wifi_log(os.path.join(cur, d), wifi_log_init)

					if len(input_file_folder) > 0:
						write_wifi_output_file(input_file_folder, wifi_data)

					os.chdir(full_scipt_path)
					os.chdir(root_path)
			#print("======================")

	except Exception as ex:
		print("Exception happened. Exception code:", ex.args)

		os.chdir(full_scipt_path)
		log_file_name = 'PBPF_error_log' + '.txt'
		logging.basicConfig(filename=log_file_name, level=logging.INFO,
							format='%(asctime)s %(levelname)s %(name)s %(message)s')
		logger = logging.getLogger(__name__)
		logger.error(ex, exc_info=True)

		raise SystemExit
