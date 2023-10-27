from inspect import getframeinfo, currentframe

def get_file_name():
    cf = currentframe()
    name = getframeinfo(cf.f_back).filename
    return name
