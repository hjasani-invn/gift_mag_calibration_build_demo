import beacon_distributor


if __name__ == "__main__":
    image_file_full_path = 'image.bmp'
    settings_file_full_path = 'venue.json'

    BD = beacon_distributor.BeaconDistributor(settings_file_full_path, image_file_full_path)

    BD.place_beacons()

