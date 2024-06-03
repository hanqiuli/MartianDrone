# Standard libraries
import math
import random
import sys

# Third party libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp

class Scanning:

    def __init__(self):
        #define different resolutions for cameras
        self.MP = {               #Resolution of Selected Camera
            'pxls' : 20 * 10 ** (6),
            'ratio' : 4 / 3,
            #'rx_hor' : int(20*10**(6)*4/7),
            #'rx_ver' : int(20*10**(6)*3/7),
            'rx_hor' : 5280,
            'rx_ver' : 3956,
            'angular_vof_h' : np.deg2rad(84),
            'focal_length' : 12.29,    #focal length mm
            'sensor_width' : 17.4,
            'sensor_length' : 13,
            'exposure_time' : 1/8000
        }
        self.MP['angular_vof_v'] = self.MP['angular_vof_h']/self.MP['ratio']


        self.MS = {
            'pxls': 5 * 10 ** (6),
            'ratio': 4 / 3,
            # rx_hor = int(pxls*4/7)
            # rx_ver = int(pxls*3/7)
            'rx_hor': 2592,
            'rx_ver': 1944,
            'angular_vof_h': np.deg2rad(61.2),
            'angular_vof_v': np.deg2rad(48.10),
            'focal_length': 4.34,  # focal length mm
            'sensor_width': 5.2,
            'sensor_length': 3.9,
            'exposure_time': 1/12800
        }
        return


    def scanning(self, h, v, scan_area, camera):
        # h         float   scanning altitude in meters
        # v         float   scanning velocity in meters
        # scan_area float   area to be scanned in m^2
        # camera    dict    dictionary including camera specs (either self.MP or self.MS)

        scan_path_width = 2 * h * np.tan(camera['angular_vof_h'] / 2)  # horizontal scan path width
        scan_path_length = 2 * h * np.tan(camera['angular_vof_v'] / 2)  # vertical scan path width
        area_length = np.sqrt(scan_area)
        scan_lines = int(np.ceil(area_length / scan_path_width))  # scan lines needed to cover area
        scan_time = area_length * scan_lines / v  # time needed to scan 1km^2 area [s]

        #Calculate resolutions: Motion blur should be lower than GSD_v
        motion_blur = v * camera['exposure_time'] * 1000  # blur in [mm]

        GSD_h = camera['sensor_width'] * h * 1000 / (camera['focal_length'] * camera['rx_hor'])  #Grounds sampling distance [mm] in the cross-path direction
        GSD_v = camera['sensor_length'] * h * 1000 / (camera['focal_length'] * camera['rx_ver']) #Grounds sampling distance [mm] in the along-path direction
        #Gather all results in a dictionary
        scanning_dict = {
            'scan_lines': scan_lines,
            'scan_time': scan_time / 60,
            'motion_blur': motion_blur,
            'GSD_horizontal': GSD_h,
            'GSD_vertical': GSD_v
        }
        return scanning_dict

def main():
    scanning = Scanning()
    birds_eye = [100, 25, 1000**2]
    validate = [217, 15, 200*10000]
    close_up = [20, 5, 100**2]
    scan_dict = scanning.scanning(*validate, scanning.MP)
    print(scan_dict)
    return scan_dict

if __name__ == '__main__':
    main()