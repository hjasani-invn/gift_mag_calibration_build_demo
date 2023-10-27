import os


def maps_list_to_dict(folder, maps_list):

    maps_dict = {}

    for one_map in maps_list:
        n = one_map.find("map") + len("map")
        #print(n)
        #m = one_map.find(".")
        m = one_map.index(".", n)
        #print(m)
        key = one_map[n:m:1]
        #print(key)
        maps_dict[key] = os.path.join(folder, one_map)
        #print(maps_dict)

    return maps_dict
