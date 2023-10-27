## @package creator_lms_data
#  This module provides dictionary merge operations
#

## merge Python dictionaries
#
# @param[in] a - dictionay to merge in
# @param[in] b - dictionay to merge from
# @param[in] path - optional internal dictionary path to merge
# @param[out] updated dictionarry a
#
def merge(a, b, path=None):
    "merges b into a"
    if path is None: path = []
    for key in b:
        if key in a:
            if isinstance(a[key], dict) and isinstance(b[key], dict):
                merge(a[key], b[key], path + [str(key)])
            elif a[key] == b[key]:
                pass # same leaf value
            else:
                a[key] = b[key]
                #raise Exception('Conflict at %s' % '.'.join(path + [str(key)]))
        else:
            a[key] = b[key]
    return a
