import re
import os
import sys
import subprocess
import multiprocessing
import tqdm
import logging
import argparse

sys.path.insert(0, '..')

def process_ModelDataToTPNConverter(input_file_folder_and_modelDataToTPNConverter):
	template = "{} {} {} {} {} {} {} {} {}"

	input_file_folder = input_file_folder_and_modelDataToTPNConverter[0]
	exe_path =input_file_folder_and_modelDataToTPNConverter[1]
	exe_name =input_file_folder_and_modelDataToTPNConverter[2]

	cmd_line = template.format(exe_name,
							   ' --input_model ', input_file_folder + '\\irl_data_mt_converter.txt',
							   ' --input_ble ', input_file_folder + '\\blp_debug_converter.log',
							   ' --input_wifi ', input_file_folder + '\\wifi_debug_converter.log',
							   ' --output_tpn ', input_file_folder + '\\tpn.dat')
	print(cmd_line)
	os.chdir(exe_path)
	f = open(os.path.join(input_file_folder, "model_converter.log"), "w")
	subprocess.call(cmd_line, stdout=f, stderr=f, shell=True)


if __name__ == "__main__":
	print("")
	#process_ModelDataToTPNConverter('1234')
	scipt_path = os.path.dirname(sys.argv[0])
	full_scipt_path = os.path.abspath(scipt_path)

	parser = argparse.ArgumentParser(description="converter parameters")

	parser.add_argument("-i", "--input_data_path", help="folder with input data")
	parser.add_argument("-c", "--converter_exe", help="full name of ModelDataToTPNConverter application")

	args = parser.parse_args()

	root_path = args.input_data_path # folder with input data
	modelDataToTPNConverter = args.converter_exe # full name of ModelDataToTPNConverter application

	exe_path = os.path.dirname(modelDataToTPNConverter)
	exe_name = os.path.basename(modelDataToTPNConverter)
	print(exe_path)
	print(exe_name)

	os.chdir(root_path)
	dir_pattern = '.*'

	folders = []
	for cur, dirs, files in os.walk(root_path):
		for d in dirs:
			if re.match(dir_pattern, d) is not None:
				path, out_venue_folder = os.path.split(cur)

				folder = os.path.join(cur, d)
				list = [f for f in os.listdir(folder) if re.match('irl_data_mt_converter.txt', f) is not None]
				if len(list) > 0:
					#print(list[0])
					print(folder)
					statinfo = os.stat(os.path.join(folder, list[0]))
					#print(statinfo.st_size)
					if statinfo.st_size > 0:
						folders.append([folder, exe_path, exe_name])
						#process_ModelDataToTPNConverter(folder)
				os.chdir(full_scipt_path)
				os.chdir(root_path)
				#print("======================")

	pcount = 2 * multiprocessing.cpu_count()
	print('pool threads: ' + str(pcount))
	pool = multiprocessing.Pool(processes=pcount)

	try:
		for _ in tqdm.tqdm(pool.imap_unordered(process_ModelDataToTPNConverter, folders, chunksize=1), total=len(folders)):
			pass
	except Exception as ex:
		print("a worker failed, aborting...")
		if hasattr(ex, 'message'):
			print(ex.message)
		else:
			print(ex)
		pool.close()
		pool.terminate()

		print("Exception happened. Exception code:", ex.args)

		os.chdir(full_scipt_path)
		log_file_name = 'PBPF_error_log' + '.txt'
		logging.basicConfig(filename=log_file_name, level=logging.INFO,
							format='%(asctime)s %(levelname)s %(name)s %(message)s')
		logger = logging.getLogger(__name__)
		logger.error(ex, exc_info=True)
	else:
		pool.close()
		pool.join()

	pool.close()
	pool.join()


