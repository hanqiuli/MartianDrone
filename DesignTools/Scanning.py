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
    rx_hor = int(pxls*4/7)
    rx_ver = int(pxls*3/7)
    return(rx_hor, rx_ver, ratio)

rx_hor,rx_ver,ratio = MP()
#print(rx_hor,rx_ver,ratio,rx_hor/rx_ver)
h     = 100 #m
AFOV_h = np.deg2rad(45) #rad (input deg)
AFOV_v = AFOV_h/ratio #rad (input deg)
v     = 10  #m/s
fps = 300 #fps
t_exp = 1/2000 #exposure time [s]

def scanning(h,v,AFOV_h,AFOV_v,t_exp):
    rx_hor, rx_ver, ratio = MP()
    FOV_h = 2* h * np.tan(AFOV_h/2)  #horizontal scan path width
    FOV_v = 2* h * np.tan(AFOV_v/2) #vertical scan path width

    scan_lin = int(np.ceil(500/FOV_h)) #scan lines needed to cover area
    scan_time = 500*scan_lin /v  #time needed to scan 1km^2 area [s]

    res_h = FOV_h/rx_hor*1000 #spatial res of image [mm]
    res_v = FOV_v/rx_ver*1000
    blur_V = v*t_exp*1000 #blur in [mm]
    res = max(res_h,res_v,blur_V)
    return(FOV_h,FOV_v,scan_lin,scan_time/60,res_h, res_v, blur_V, h, v, AFOV_h, AFOV_v, t_exp)

FOV_h,FOV_v,scan_lin,scan_time,res_h, res_v, blur_V,h,v, AFOV_h, AFOV_v, t_exp = scanning(h,v,AFOV_h,AFOV_v,t_exp)
print(scanning(h,v,AFOV_h,AFOV_v,t_exp))
def scanning2(h,v,AFOV_h,AFOV_v,t_exp):
    rx_hor, rx_ver, ratio = res_8k()
    FOV_h = 2* h * np.tan(AFOV_h/2)  #horizontal scan path width
    FOV_v = 2* h * np.tan(AFOV_v/2) #vertical scan path width

    scan_lin = int(np.ceil(100/FOV_h)) #scan lines needed to cover area
    scan_time = 100*scan_lin /v  #time needed to scan 1km^2 area [s]

    res_h = FOV_h/rx_hor*1000 #spatial res of image [mm]
    res_v = FOV_v/rx_ver*1000
    blur_V = v*t_exp*1000 #blur in [mm]
    res = max(res_h,res_v,blur_V)
    return(scan_time,res)
#print(scanning(h,v,AFOV_h,AFOV_v,t_exp))


def optimize():
    #FOV_h,FOV_v,scan_lin,scan_time,res,h,v, AFOV_h, AFOV_v, t_exp = scanning(h,v,AFOV_h,AFOV_v,t_exp)
    t_max = 30*60
    h_range = np.arange(5,300, 1)
    v_range = np.arange(1, 30, 0.5)
    FOV_range = np.linspace(0, AFOV_h, 50)
    exp_range = np.linspace(1/12000, 1/500, 500)

    H, V = np.meshgrid(h_range, v_range)
    print(H.shape, V.shape)
    scan = np.zeros(H.shape)
    resolution = np.zeros(H.shape)
    index = np.array([], dtype=int)
    for i in range(H.shape[0]):
        for j in range(H.shape[1]):
            scan[i,j],resolution[i,j] = scanning2(H[i,j],V[i,j],AFOV_h,AFOV_v,t_exp)
            if scan[i,j] < 30*60 and resolution[i,j] <=5:
                index = np.append(index,[i,j])
    return
#optimize()

def scanning3():
    rx_hor, rx_ver, ratio = MP()
    h = np.arange(0,300, 1)
    AFOV_h = np.deg2rad(25)  # rad (input deg)
    AFOV_v = AFOV_h / ratio  # rad (input deg)
    t_exp = 1/2000
    resolution = 5 #resolution [mm]
    rx_hor, rx_ver, ratio = res_8k()
    FOV_h = 2 * h * np.tan(AFOV_h / 2)  # horizontal scan path width
    FOV_v = 2 * h * np.tan(AFOV_v / 2)  # vertical scan path width
    V_max = resolution/(t_exp*1000) #max velocity with spatial resolution [m/s
    scan_lin = int(np.ceil(1000 / FOV_h))  # scan lines needed to cover area
    scan_time = 1000 * scan_lin / V_max  # time needed to scan 1km^2 area [s]

    res_h = FOV_h / rx_hor * 1000  # spatial res of image [mm]
    res_v = FOV_v / rx_ver * 1000




    return (scan_time, res)

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


