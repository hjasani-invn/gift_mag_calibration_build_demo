import sys
import logging
from sys import platform
import all_venues_test
import convert_path_to_linux


if __name__ == "__main__":

    try:
        input_folder = sys.argv[1]
        output_folder = sys.argv[2]

        if platform == 'linux' or platform == 'linux2':
            print('Running tests under linux')
            input_folder = convert_path_to_linux.convert_path(input_folder)
            output_folder = convert_path_to_linux.convert_path(output_folder)

        print('input folder:', input_folder)
        print('output folder:', output_folder)

        all_venues_test.run_test(input_folder, output_folder)

    except Exception as ex:

        print("Exception happened. Exception code:", ex.args)
        log_file_name = 'ASCA_Tests_Error_log' + '.txt'
        logging.basicConfig(filename=log_file_name, level=logging.INFO,
                            format='%(asctime)s %(levelname)s %(name)s %(message)s')
        logger = logging.getLogger(__name__)
        logger.error(ex, exc_info=True)

        raise SystemExit
