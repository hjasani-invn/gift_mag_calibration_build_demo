import os
import main
import contextlib


def run_test(input_folder, output_folder):
    list_subfolders_with_paths = [f.path for f in os.scandir(input_folder) if f.is_dir()]

    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    for subfolder_path in list_subfolders_with_paths:
        one_test_out_folder = os.path.basename(os.path.normpath(subfolder_path))
        one_test_out_full_path = os.path.join(output_folder, one_test_out_folder)
        if not os.path.exists(one_test_out_full_path):
            os.makedirs(one_test_out_full_path)

        try:
            with open(os.devnull, 'w') as devnull:
                with contextlib.redirect_stdout(devnull):
                    main.run_main(subfolder_path, one_test_out_full_path)

        except:
            print('ASCA fails at test: ', one_test_out_folder)
            pass




