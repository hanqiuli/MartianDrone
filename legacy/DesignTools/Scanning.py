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

rx_hor,rx_ver,ratio = res_8k()
h     = 50 #m
AFOV_h = np.deg2rad(12.5) #rad (input deg)
AFOV_v = AFOV_h/ratio #rad (input deg)
v     = 15  #m/s
fps = 300 #fps
t_exp = 1/12000 #exposure time [s]

def scanning(h,v,AFOV_h,AFOV_v,t_exp):
    rx_hor, rx_ver, ratio = res_8k()
    FOV_h = 2* h * np.tan(AFOV_h/2)  #horizontal scan path width
    FOV_v = 2* h * np.tan(AFOV_v/2) #vertical scan path width

    scan_lin = int(np.ceil(100/FOV_h)) #scan lines needed to cover area
    scan_time = 100*scan_lin /v  #time needed to scan 1km^2 area [s]

    res_h = FOV_h/rx_hor*1000 #spatial res of image [mm]
    res_v = FOV_v/rx_ver*1000
    blur_V = v*t_exp*1000 #blur in [mm]
    res = max(res_h,res_v,blur_V)
    return(FOV_h,FOV_v,scan_lin,scan_time,res, h, v)

FOV_h,FOV_v,scan_lin,scan_time,res,h,v = scanning(h,v,AFOV_h,AFOV_v,t_exp)

print(scanning(h,v,AFOV_h,AFOV_v,t_exp))




def LoS(h,d):
    R_mars = 3396.2    #km
    h = h/1000
    d = d
    p = 2 * np.pi * R_mars
    theta = d/p * 2*np.pi
    x_d = 0
    y_d = R_mars+h
    x_gs = -R_mars*np.sin(theta)
    y_gs = R_mars*np.cos(theta)
    dx = x_gs - x_d
    dy = y_gs - y_d
    return theta, x_d, y_d, x_gs, y_gs


