import numpy as np
from matplotlib import pyplot as plt


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

def MP():
    pxls = 20*10**(6)
    ratio = 4/3
    #rx_hor = int(pxls*4/7)
    #rx_ver = int(pxls*3/7)
    rx_hor = 5280
    rx_ver = 3956
    return(rx_hor, rx_ver, ratio)

def MS():
    pxls = 5*10**6
    rx_hor = 2592
    rx_ver = 1944
    ratio = rx_hor / rx_ver
    return (rx_hor, rx_ver, ratio)

rx_hor,rx_ver,ratio = MP()

#print(rx_hor,rx_ver,ratio,rx_hor/rx_ver)
h     = 50 #m
AFOV_h = np.deg2rad(84) #rad (input deg)
AFOV_v = AFOV_h/ratio #rad (input deg)
v     = 10  #m/s
fps = 60 #fps
t_exp = 1/8000 #exposure time [s]

def scanning(h,v,AFOV_h,AFOV_v,t_exp, Area_length):
    rx_hor, rx_ver, ratio = MS()
    FOV_h = 2* h * np.tan(AFOV_h/2)  #horizontal scan path width
    FOV_v = 2* h * np.tan(AFOV_v/2) #vertical scan path width
    #Area_length = 1000
    scan_lin = int(np.ceil(Area_length/FOV_h)) #scan lines needed to cover area
    scan_time = Area_length*scan_lin /v  #time needed to scan 1km^2 area [s]

    res_h = FOV_h/rx_hor*1000 #spatial res of image [mm]
    res_v = FOV_v/rx_ver*1000
    blur_V = v*t_exp*1000 #blur in [mm]
    res = max(res_h,res_v,blur_V)
    #w = 17.4 #detector width [mm]
    #f = 12.29 #focal length [mm]
    w = 5.2  # detector width [mm]
    f = 4.34 #focal length [mm]
    GSD = w*h*1000/(f*rx_hor)
    scanning_dict = {
        'rx_hor':rx_hor,
        'rx_ver':rx_ver,
        'ratio':ratio,
        'FOV_h':FOV_h,
        'FOV_v':FOV_v,
        'scan_lin':scan_lin,
        'scan_time': scan_time/60,
        'res_h':res_h,
        'res_v':res_v,
        'blur':blur_V,
        'max res': res,
        'GSD': GSD,
        'Area_length' : Area_length,
        'h': h,
        'V': v,
        'AFOV_h':AFOV_h,
        'AFOV_v':AFOV_v,
        't_exp':t_exp
        }
    return scanning_dict





def optimize():
    #FOV_h,FOV_v,scan_lin,scan_time,res,h,v, AFOV_h, AFOV_v, t_exp = scanning(h,v,AFOV_h,AFOV_v,t_exp)
    t_max = 30*60
    h_range = np.arange(5,300, 1)
    v_range = np.arange(1, 30, 0.5)
    h = 100 # m
    #AFOV_h = np.deg2rad(84)  # rad (input deg)
    #AFOV_v = AFOV_h / ratio  # rad (input deg)
    AFOV_h = np.deg2rad(61.2)  # rad (input deg)
    AFOV_v = np.deg2rad(48.10)     # rad (input deg)
    v = 25  # m/s
    Area_length = 1000
    print(Area_length**2)
    t_exp = 1 / 12800  # exposure time [s]
    scanning_dict = scanning(h,v,AFOV_h,AFOV_v,t_exp, Area_length)
    for param, value in scanning_dict.items():
        print(f"{param}: {value}")
    H, V = np.meshgrid(h_range, v_range)
    #print(H.shape, V.shape)
    scan = np.zeros(H.shape)
    resolution = np.zeros(H.shape)
    index = np.array([], dtype=int)

    return

optimize()

def function():
    return


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


