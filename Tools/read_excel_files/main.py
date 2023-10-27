import os
import sys
import re
import logging
import openpyxl    #reading/writing Excel table

import read_excel_files

if __name__ == "__main__":
    try:
        script_path = os.path.dirname(sys.argv[0])
        full_script_path = os.path.abspath(script_path)

        input_folder = sys.argv[1]
        #output_folder = sys.argv[2]
        #output_file =  sys.argv[2].replace("\r", "")
        #print("output file is ", output_file)

        os.chdir(full_script_path)

        excel_pattern = '(.*\.xlsx)'

        input_file_list = []

        for cur, dirs, files in os.walk(input_folder):
            for f in files:
                print('file name: ', f)
                if re.match(excel_pattern, f):
                    input_file_list.append(os.path.join(cur, f))

        wb_out = openpyxl.Workbook()
        sheet_out = wb_out.active
        sheet_out.title = 'KPI results'
        cur_row = 1

        print('')
        print('=============================')
        print('')

        for excel_file in input_file_list:
            cur_row = read_excel_files.process_all_sheets(excel_file, wb_out, cur_row )

        filename = 'kpi_results' + '.xlsx'
        wb_out.save(filename)

        print('')
        print('=============================')
        print('')
        with open('kpi_results.csv', 'w') as f:
            for row in sheet_out.rows:
                for cell in row:
                    if cell.value == None:
                        #print(',', end=' ')
                        f.write(',')
                    else:
                        if isinstance(cell.value, float):
                            #print('%.3f' % (cell.value), end=',')
                            f.write('%.3f %s' % (cell.value, ','))
                        else:
                            #print(cell.value, end=',')
                            if isinstance(cell.value, int):
                                f.write('%d %s' % (cell.value, ','))
                            else:
                                f.write(cell.value + ',')
                #print('')
                f.write('\n')

    except Exception as ex:
        print("Exception happened. Exception code:", ex.args)

        log_file_name = 'read_excel_files_error_log' + '.txt'
        logging.basicConfig(filename=log_file_name, level=logging.INFO,
                            format='%(asctime)s %(levelname)s %(name)s %(message)s')
        logger = logging.getLogger(__name__)
        logger.error(ex, exc_info=True)

        raise SystemExit
