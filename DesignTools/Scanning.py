import numpy as np

def res_8k():
    rx_hor = 7680
    rx_ver = 4320
    ratio = rx_hor/rx_ver
    return(rx_hor,rx_ver,ratio)

def res_4k():
    rx_hor = 3840
    rx_ver = 2160
    ratio = rx_hor/rx_ver
    return(rx_hor,rx_ver,ratio)

def res_HD():
    rx_hor = 1920
    rx_ver = 1080
    ratio = rx_hor / rx_ver
    return (rx_hor, rx_ver, ratio)

#rx_hor,rx_ver,ratio = res_8k()
h     = 100 #m
AFOV_h = np.deg2rad(25) #rad (input deg)
AFOV_v = np.deg2rad(12.5) #rad (input deg)
v     = 20  #m/s

def scanning(h,v,AFOV_h,AFOV_v):
    rx_hor, rx_ver, ratio = res_8k()
    FOV_h = 2* h * np.tan(AFOV_h/2)  #horizontal scan path width
    FOV_v = 2* h * np.tan(AFOV_v/2) #vertical scan path width

    scan_lin = int(np.ceil(1000/FOV_h)) #scan lines needed to cover area
    scan_time = 1000*scan_lin /v  #time needed to scan 1km^2 area [s]

    res_h = FOV_h/rx_hor*1000 #spatial res of image [mm]
    res_v = FOV_v/rx_ver*1000
    return(FOV_h,FOV_v,scan_lin,scan_time,res_h,res_v, h, v)

FOV_h,FOV_v,scan_lin,scan_time,res_h,res_v,h,v = scanning(h,v,AFOV_h,AFOV_v)

#print(scanning(h,v,AFOV_h,AFOV_v))