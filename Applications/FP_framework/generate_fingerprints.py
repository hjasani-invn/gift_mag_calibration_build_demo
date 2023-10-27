__author__ = 'kotik'

# class FingerprintsGenerator calls FP building console application
# is initialized with fp_settings.txt, also "main_settings" parameter is passed to get 'tests_path'

import subprocess
import os
import shutil

class FingerprintsGenerator(object):

    def __init__(self, settings):
        self.settings = settings

    def call_generator(self, main_settings):
        # calling from main folder
        # FP_builder = 'FP_builder.exe '
        FP_builder = main_settings['FP_builder_name'] + ' '
        FP_builder += '--settings '
        FP_builder += main_settings['json_FP_builder']

        print(FP_builder)

        return_code = subprocess.call(FP_builder, shell=True)

        # TODO: new JSON parsing here, check that it works
        mfp_db = self.settings['name']+'.mfp3'

        if main_settings['integration_testing'] == 'ON':
            fp_files = [ mfp_db ]
            for f in fp_files:
                if os.path.isfile(f):
                    shutil.copy(f, main_settings['tests_path']+f)


