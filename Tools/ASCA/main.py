import os
import sys
import re
import process_asca
import logging


def run_main(input_path, output_path):
    output_path = output_path.replace("\r", "")

    blebase_pattern = r'(.*\.ble4)'
    wifibase_pattern = r'(.*\.wifi4)'
    magbase_pattern = r'(.*\.mfp4)'

    settings_pattern = r'(.*venue\.json)'

    map_pattern = r'(.*map-?\d+\.geojson)'

    patterns = []
    patterns.append(blebase_pattern)
    patterns.append(wifibase_pattern)
    patterns.append(magbase_pattern)
    patterns.append(settings_pattern)

    input_file_list = []

    for pattern in patterns:
        file_list = [f for f in os.listdir(input_path) if re.match(pattern, f) is not None]
        if len(file_list) > 0:
            input_file_list.append(os.path.join(input_path, file_list[0]))

    maps_files = [f for f in os.listdir(input_path) if re.match(map_pattern, f) is not None]

    for map_file in maps_files:
        input_file_list.append(os.path.join(input_path, map_file))
    print(input_file_list)
    print("output folder = ", output_path)
    result = process_asca.process_asca(output_path, input_file_list)
    print(result)


if __name__ == "__main__":

    try:

        input_folder = sys.argv[1]
        output_folder = sys.argv[2]

        run_main(input_folder, output_folder)

    except Exception as ex:
        print("Exception happened. Exception code:", ex.args)

        log_file_name = 'ASCA_error_log' + '.txt'
        logging.basicConfig(filename=log_file_name, level=logging.INFO,
                            format='%(asctime)s %(levelname)s %(name)s %(message)s')
        logger = logging.getLogger(__name__)
        logger.error(ex, exc_info=True)

        raise SystemExit
