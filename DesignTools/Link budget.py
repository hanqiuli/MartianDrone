import numpy as np
from Data_budget import data

def dB(n):
    return 10*np.log10(n)



DR, table = data()
Pt = 10  #W  transmitting power
f = 12      #GHz  transmitting frequency
B = DR/2/1  #bandwith [Hz]
R = 65     #km
Ll = 0.8   #Link loss
Gt = 5     #transmission gain
EIRP = Pt * Ll * Gt
La = 10**(-4)*R
Ls = 147.55 - 2*dB(R*1000) - 2*dB(f*10**(9)) #space loss [dB]
Ts = 273.15+50 #system noise T [K]
Kb = 1.380*10**(-23)
D_rec = 2
A_rec = np.pi*(D_rec/2)**2
n_rec = 0.7
Gr = np.pi**2*D_rec**2*n_rec / (3*10**8/(f*10**9))


Wf = EIRP * 4*np.pi*R**2
C = Wf * A_rec * n_rec
SNR = dB(EIRP) + Ls + La + dB(Gr) - dB(Kb) - dB(Ts) - dB(DR)   #SNR dB
Eb = C/DR   #Bit energy
N = Kb * Ts * B  #Noise

print(SNR, 10**(SNR/10))