import compare_wfp
import sys


if __name__ == "__main__":
    settings_json = sys.argv[1]
    wfp_1 = sys.argv[2]  #.wifi4 format
    wfp_2 = sys.argv[3]  #.wifi4 format

    print(settings_json)
    print(wfp_1)
    print(wfp_2)
    print("")

    floor_number = int(sys.argv[4])  # logical number of floor
    print("floor, ", floor_number)
    print("")

    compare_wfp.compare_wifi_fingerprints(settings_json, wfp_1, wfp_2, floor_number)
