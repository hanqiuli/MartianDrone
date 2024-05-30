import numpy as np
from Scanning import scanning
from Scanning import res_8k,res_4k,res_HD
from tabulate import tabulate

h     = 100 #m
AFOV_h = np.deg2rad(25) #rad (input deg)
AFOV_v = np.deg2rad(12.5) #rad (input deg)
v     = 20  #m/s
t_exp = 1/12000
def data_scan():
    h = 100  # m
    AFOV_h = np.deg2rad(25)  # rad (input deg)
    AFOV_v = np.deg2rad(12.5)  # rad (input deg)
    v = 20  # m/s
    rx_hor, rx_ver, ratio = res_8k()

    FOV_h, FOV_v, scan_lin, scan_time, res, h, v = scanning(h, v, AFOV_h, AFOV_v,t_exp)

    px = rx_hor*rx_ver #pixels per image
    b_px = 24   #bit/pixel
    data_pic = px*b_px #bits per image
    N_pics_min = 2* np.ceil(1000/FOV_v*scan_lin) #total pictures min, that need to be taken (2 factor to cover everything twice)
    fps = 60    #frames per second of the camera
    N_pics = fps*scan_time  #pictures taken based on fps and scan time
    CF  = 20    #compression factor
    Data_scan = data_pic*N_pics_min/CF #total data [bits]
    trans_time = scan_time*1.5 #transmission time, 1.5 factor for longer time to transmit images
    data_rate = Data_scan/trans_time
    return Data_scan,data_rate

#print(data_scan())

def data_sens():
    DOF = 6
    v_vec = 6
    a_vec = 6
    T =1
    P =1
    rho = 1
    time = 124
    bit_pr = 64 #floating point
    sampling_rate = 1600 #Hz
    data_rate = (DOF+v_vec+a_vec)*bit_pr*sampling_rate + (T+P+rho+time)*bit_pr*60
    return data_rate

def data():
    total_Data_scan,data_rate_scan = data_scan()
    data_rate_sens = data_sens()
    other = 0.25*(data_rate_scan+data_rate_sens)
    data = data_rate_scan+data_rate_sens+other
    table = [['','bits/s', 'Mbit/s'],
             ['scanning', data_rate_scan, data_rate_scan*10**(-6)],
             ['sensor data',data_rate_sens, data_rate_sens*10**(-6)],
             ['other', other, other*10**(-6)],
             ['total', data, data*10**(-6)]]
    #print(tabulate(table, headers='firstrow', tablefmt='fancy_grid'))
    return data, table

data()