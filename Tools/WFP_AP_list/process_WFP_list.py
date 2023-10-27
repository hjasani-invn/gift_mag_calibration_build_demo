import os
import re
import json
import logging
import logging
import WFPdata

def process(out_folder, input_file_list):

    print("")
    version = "1.0.0"
    print("WiFi AP list tool, version ", version)
    print("")

    try:
        #settings_pattern = '(.*\.json)'
        wifibase3_pattern = '(.*\.wifi3)'
        wifibase4_pattern = '(.*\.wifi4)'


        #list = [f for f in input_file_list if re.match(settings_pattern, f) is not None]
        #if len(list) > 0 :
        #    settings_json_file = list[0]
        #    settings = json.load(open(settings_json_file))
        #else :
        #    settings_json_file = ""
        #    finish_flag = True

        floor_count = 1 #settings['venue']['floors_count']
        floor_shift = 1
        floor_zero_enable = False

        #if 'floor_shift' in settings['venue']:
        #    floor_shift = settings['venue']['floor_shift']
        #if 'floor_zero_enable' in settings['venue']:
        #    floor_zero_enable = settings['venue']['floor_zero_enable']

        list = [f for f in input_file_list if re.match(wifibase3_pattern, f) is not None]
        if len(list) > 0 :
            WFP_db3_file = list[0]
        else :
            WFP_db3_file = ""

        list = [f for f in input_file_list if re.match(wifibase4_pattern, f) is not None]
        if len(list) > 0 :
            WFP_db4_file = list[0]
        else :
            WFP_db4_file = ""

        if not os.path.isfile(WFP_db3_file):
            print('WiFi3 base is not found')
            has_wifi_db3 = False
        else:
            print('WiFi3 base is ', WFP_db3_file)
            has_wifi_db3 = True
        if not os.path.isfile(WFP_db4_file):
            print('WiFi4 base is not found')
            has_wifi_db4 = False
        else:
            print('WiFi4 base is ', WFP_db4_file)
            has_wifi_db4 = True

        if (has_wifi_db3 or has_wifi_db4) == False:
            raise SystemExit

        # create out folder
        if not os.path.isdir(out_folder):
            os.mkdir(out_folder)

        out_file = out_folder + '/' + 'WFP_AP_list.txt'
        fo = open(out_file , 'w')

        for floor in range(0, floor_count):
            WiFi3_AP_list, WiFi4_AP_list = WFPdata.get(has_wifi_db3, WFP_db3_file, has_wifi_db4, WFP_db4_file, floor)
            #floor_str = 'floor: ' + str(floor)
            #print(floor_str)
            #fo.write('%s\n' % floor_str)
            if len(WiFi3_AP_list) > 0:
                fo.write('%s\n' % 'WiFi3_AP3_list:')
                fo.write('%s\n' % ' ')
                #for WiFi_AP in WiFi3_AP_list:
                #    print(WiFi_AP)
                fo.writelines('%s\n' % WiFi_AP for WiFi_AP in WiFi3_AP_list)
                fo.write('%s\n' % ' ')

            if len(WiFi4_AP_list) > 0:
                fo.write('%s\n' % 'WiFi4_AP4_list:')
                fo.write('%s\n' % ' ')
                #for WiFi_AP in WiFi4_AP_list:
                #    print(WiFi_AP)
                fo.writelines('%s\n' % WiFi_AP for WiFi_AP in WiFi4_AP_list)
                fo.write('%s\n' % ' ')
        fo.close()

    except Exception as ex:
        print("Exception happened. Exception code:", ex.args)

        log_file_name = 'WiFi3_AP_list_error_log' + '.txt'
        logging.basicConfig(filename=log_file_name, level=logging.INFO,
                            format='%(asctime)s %(levelname)s %(name)s %(message)s')
        logger = logging.getLogger(__name__)
        logger.error(ex, exc_info=True)

        raise SystemExit


