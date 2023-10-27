import compare_wfp
import sys


if __name__ == "__main__":
    settings_json = sys.argv[1]
    wfp_1 = sys.argv[2]  # file to be modified - .wifi4 format
    wfp_2 = sys.argv[3]  # file using for modification - .wifi4 format
    wfp_3 = sys.argv[4]  # new file - .wifi4 format

    #settings_json = 'c:/Users/vpentyukhov/Development/InvenSenseInc/Gift/Tools/ASCA/venues_/ICA_crowdsource/venue.json'
    #wfp_1 = '//cayyc-proj01/compute02/vpentyukhov/fingerprints/ICA_crowdsource/ICA_CrowdSource.wifi4'
    #wfp_2 = 'c:/Users/vpentyukhov/Development/InvenSenseInc/Gift/Applications/fp_builder.console/venues/ICA_crowdsource/fp_1m/ICA_CrowdSource.wifi4'
    #floor_number = 5

    compare_wfp.compare_wifi_fingerprints(settings_json, wfp_1, wfp_2, wfp_3)
