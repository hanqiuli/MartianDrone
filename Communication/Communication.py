# Standard libraries
import math
import random
import sys

# Third party libraries
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
from tabulate import tabulate

from Scanning import Scanning
scanning = Scanning.Scanning()
class Data:
    def __init__(self):

        self.h = 100
        self.v = 25
        return

    def data_scan(self):
        area = 1000**2

        camera = scanning.MP
        scan_dict = scanning.scanning(self.h,self.v,area, camera)

        b_px = 12  # bit/pixel
        data_pic = camera['pixels'] * b_px  # bits per image
        N_pics_min = np.ceil(np.sqrt(area) / (scan_dict['scan_path_length']*0.7) * scan_dict['scan_lines'])  # total pictures min, that need to be taken (2 factor to cover everything twice)

        CF = 250  # compression factor
        Data_scan = data_pic * N_pics_min / CF  # total data [bits]
        trans_time = scan_dict['scan_time'] * 2  # transmission time, 1.5 factor for longer time to transmit images
        data_rate = Data_scan / trans_time
        return Data_scan, data_rate

    def data_sens(self):
        DOF = 6
        v_vec = 6
        a_vec = 6
        T = 1
        P = 1
        rho = 1
        time = 124
        bit_pr = 64  # floating point
        sampling_rate = 1600  # Hz
        data_rate = (DOF + v_vec + a_vec) * bit_pr * sampling_rate + (T + P + rho + time) * bit_pr * 60
        return data_rate
    def total_data(self):
        total_Data_scan, data_rate_scan = self.data_scan()
        data_rate_sens = self.data_sens()
        #print(f'Data rate is: {data_rate_scan*10**(-6)} Mbps')
        other = 0.25 * (data_rate_scan + data_rate_sens)
        data = data_rate_scan + data_rate_sens + other
        table = [['', 'bits/s', 'Mbit/s'],
                 ['scanning', data_rate_scan, data_rate_scan * 10 ** (-6)],
                 ['sensor data', data_rate_sens, data_rate_sens * 10 ** (-6)],
                 ['other', other, other * 10 ** (-6)],
                 ['total', data, data * 10 ** (-6)]]
        #print(tabulate(table, headers='firstrow', tablefmt='fancy_grid'))
        return data

class Communication:
    def __init__(self):
        data = Data()
        self.comm_dict = {
            'C_N_min' : 100 * data.total_data(),                #Minimum required C/N0 ratio
            'Pt' : 1,                                           #Transmitted Power [W]
            'Gt' : 1,                                           #Transmitter Gain [-]
            'f' : 937,                                          #Transmitting frequency [MHz]
            'Gr' : 1,                                           #Receiver Gain [-]
            'Ts' : 10 ** (27.4 / 10),                           #System Noise Temperature [K]
            'Kb' : 1.380 * 10 ** (-23),                         #Boltzmann Constant
            'R' : 30,                                           #Transmission distance [km]
        }
        self.comm_dict['lambda']: 3 * 10 ** 8 / (f * 10 ** 6)   #Transmission Wavelength [m]
        return
    def dB(self,n):
        #Convert a number into deciBells
        #n type: int,float,list,array (1D)
        if type(n) == int or type(n) == float:
            dB = 10 * np.log10(n)
            return dB
        else:
            dB = np.zeros(len(n))
            for i in range(len(n)):
                if n[i] <= 0:
                    dB[i] = -np.inf
                else:
                    dB[i] = 10 * np.log10(1000 * n[i])
                return dB
        return

    def invdB(self,n):
        #Convert deciBell number into normal number
        if type(n) == int or type(n) == float:
            invdB = 10**(n/10)
            return invdB
        else:
            invdB = np.zeros(len(n))
            for i in range(len(n)):
                invdB[i] = 10**(n[i]/10)
            return invdB
        return


    def W_to_dBm(self,n):
        #Convert power in Watts into dBm
        #n type: int,float,list,array (1D)
        if type(n) == int or type(n) == float:
            if n<=0:
                dbm = -np.inf
            elif n>0:
                dbm = 10 * np.log10(1000 * n)
            return dbm
        else:
            dbm = np.zeros(len(n))
            for i in range(len(n)):
                if n[i] <= 0:
                    dbm[i] = -np.inf
                else:
                    dbm[i] = 10 * np.log10(1000 * n[i])
                return dbm
        return
    def dBm_to_W(self,n):
        #Convert value in dBm into Watts
        #Types: int,float,list,array (1D)
        if type(n) == int or type(n) == float:
            W = 10 ** (n / 10) / 1000
            return W
        else:
            W = np.zeros(len(n))
            for i in range(len(n)):
                W[i] = 10 ** (n[i] / 10) / 1000
            return W
        return

    def C_N(self):
        C_N = self.comm_dict['Pt'] * self.comm_dict['Gt'] * (self.comm_dict['lambda'] / (4 * np.pi * self.comm_dict['R'] * 1000)) ** 2 * (self.comm_dict['Gr'] / self.comm_dict['Ts']) * 1 / self.comm_dict['Kb']
        return C_N

    def Range(self):
        range = self.comm_dict['lambda'] / (4*np.pi)  * np.sqrt((self.comm_dict['Pt'] * self.comm_dict['Gt'] * (self.comm_dict['Gr'] / self.comm_dict['Ts'])) / (self.comm_dict['C_N_min'] * self.comm_dict['Kb']))
        return range


def main():
    data = Data()
    communication = Communication()


    data.total_data()
    return



if __name__ == '__main__':
    main()