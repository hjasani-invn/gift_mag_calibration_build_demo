import json
import os
import sys
import csv
import io

def read_json_file (json_file_name):

    json_file = open(json_file_name, 'r')
    json_text = json_file.read()
    #print(json_text)
    json_data = json.loads(json_text)
    json_file.close()

    return json_data



