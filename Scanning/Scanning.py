# Standard libraries
import math
import random
import sys

# Third party libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
import tabulate

class Scanning:

    def __init__(self):
        #define different resolutions for cameras
        self.MP = {               #Resolution of Selected Camera
            'pixels' : 20 * 10 ** (6),
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
            'pixels': 512*512,
            'ratio': 1,
            # rx_hor = int(pxls*4/7)
            # rx_ver = int(pxls*3/7)
            'rx_hor': 512,
            'rx_ver': 512,
            'angular_vof_h': np.deg2rad(79.7),
            'angular_vof_v': np.deg2rad(79.7),
            'focal_length': 8,  # focal length mm
            'sensor_width': 10.24,
            'sensor_length': 10.24,
            'exposure_time': 1/12800
        }
        self.RGB = {
            'pixels': 5*10**6,
            'ratio': 2448/2048,
            # rx_hor = int(pxls*4/7)
            # rx_ver = int(pxls*3/7)
            'rx_hor': 2448,
            'rx_ver': 2048,
            'angular_vof_h': np.deg2rad(102.4),
            'angular_vof_v': np.deg2rad(82.3),
            'focal_length': 3.5,  # focal length mm
            'sensor_width': 6.71,
            'sensor_length': 5.61,
            'exposure_time': 1 / 12800
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
        scan_lines = int(np.ceil(area_length / (0.5*scan_path_width)))  # scan lines needed to cover area
        scan_time = area_length * scan_lines / v  # time needed to scan 1km^2 area [s]

        #Calculate resolutions: Motion blur should be lower than GSD_v
        motion_blur = v * camera['exposure_time'] * 1000  # blur in [mm]

        GSD_h = camera['sensor_width'] * h * 1000 / (camera['focal_length'] * camera['rx_hor'])  #Grounds sampling distance [mm] in the cross-path direction
        GSD_v = camera['sensor_length'] * h * 1000 / (camera['focal_length'] * camera['rx_ver']) #Grounds sampling distance [mm] in the along-path direction
        #Gather all results in a dictionary
        scanning_dict = {
            'scan_path_width': scan_path_width,
            'scan_path_length': scan_path_length,
            'scan_lines': scan_lines,
            'scan_time': scan_time / 60,
            'motion_blur': motion_blur,
            'GSD_horizontal': GSD_h,
            'GSD_vertical': GSD_v
        }

        return scanning_dict

def main():
    scanning = Scanning()
    birds_eye = [100, 38, 1000**2]
    validate = [217, 15, 200*10000]
    close_up = [10, 2.5, 25**2]

    def total_scan_time():
        scan_dict_MP_high = scanning.scanning(*birds_eye, scanning.RGB)
        scan_dict_MS_high = scanning.scanning(*birds_eye, scanning.MS)
        print(scan_dict_MP_high)
        print(scan_dict_MS_high)
        scan_dict_birds_eye = dict.fromkeys(scan_dict_MP_high.keys(), 0)
        for key in scan_dict_birds_eye:
            scan_dict_birds_eye[key] = max(scan_dict_MP_high[key], scan_dict_MS_high[key])
        scan_dict_MP_low = scanning.scanning(*close_up, scanning.RGB)
        scan_dict_MS_low = scanning.scanning(*close_up, scanning.MS)
        print(scan_dict_MP_low)
        print(scan_dict_MS_low)
        scan_dict_close_up = dict.fromkeys(scan_dict_MP_low.keys(), 0)
        for key in scan_dict_close_up:
            scan_dict_close_up[key] = max(scan_dict_MP_low[key], scan_dict_MS_low[key])
        print(scan_dict_birds_eye)
        print(scan_dict_close_up)
        total_scan_time = scan_dict_birds_eye['scan_time'] + scan_dict_close_up['scan_time']
        print(f'Total scan time: {np.floor(total_scan_time)} minutes and {(total_scan_time-np.floor(total_scan_time))*60} seconds')
        return
    total_scan_time()

    def optimize_scanning():
        h = np.arange(1,200,1)
        v = np.arange(1,38,0.1)
        scan_time = 100
        scan_v = 0
        scan_h = 0
        scan_res = 0
        for i in range(len(h)):
            for j in range(len(v)):
                scan_dict = scanning.scanning(h[i], v[j], 1000**2, scanning.MS)
                if scan_dict['scan_time'] < scan_time and scan_dict['motion_blur'] < 0.95*scan_dict['GSD_vertical']:
                    scan_h = h[i]
                    scan_v = v[j]
                    scan_time = scan_dict['scan_time']
                    scan_res = scan_dict['GSD_vertical']
        return scan_h,scan_v,scan_time, scan_res
    #print(optimize_scanning())
    return

if __name__ == '__main__':
    main()