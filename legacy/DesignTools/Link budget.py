import numpy as np
from Data_budget import data
import matplotlib.pyplot as plt

def dB(n):
    return 10*np.log10(n)



DR, table = data()
Pt = 1  #W  transmitting power
f = 12      #GHz  transmitting frequency
lambd = 3*10**8/(f*10**9)
B = DR/2/1  #bandwith [Hz]
R = 20     #km
Ll = 0.8   #Link loss
Gt = 1     #transmission gain
EIRP = Pt * Ll * Gt
La = 10**(-4)*R #dB
Ls = 147.55 - 2*dB(R*1000) - 2*dB(f*10**(9)) #space loss [dB]
Ts = 10**(27.4/10) #system noise T [K]

Kb = 1.380*10**(-23)
D_rec = 0.5
A_rec = np.pi*(D_rec/2)**2
n_rec = 0.7
Gr = np.pi**2*D_rec**2*n_rec / lambd
Gr = 1
print(10**(La/10), Ts)
Wf = EIRP * 4*np.pi*R**2
C = Wf * A_rec * n_rec
SNR = dB(EIRP) + Ls + La + dB(Gr) - dB(Kb) - dB(Ts) - dB(DR)   #SNR dB
Eb = C/DR   #Bit energy
N = Kb * Ts * B  #Noise
CN_min = 100*DR
print(CN_min)

C_N = Pt*Gt*(lambd/(4*np.pi*R*1000))**2*(Gr/Ts)*1/Kb

#range = 4*np.pi/lambd * np.sqrt((Pt*Gt*(Gr/Ts))/(CN_min*Kb))
#print(range)

#P_lst = np.arange(1*10**(-5), 0.1, 1*10**(-5))
P_lst = np.geomspace(1e-5, 0.5, 10000)
dBm_lst = 10*np.log10(1000*P_lst)
C_N_P = P_lst*Gt*(lambd/(4*np.pi*R*1000))**2*1/(10**(La/10))*(Gr/Ts)*1/Kb
plt.plot(P_lst, C_N_P)
plt.title('C/N_0 as function of Transmission Power. Range = 30km ')
plt.xlabel('Transmitter Power [W]')
plt.ylabel('C/N_0 [-]')
#plt.savefig('Figures/C_N_P')
def W_to_dBm(n):
    dbm = np.zeros(len(n))
    for i in range(len(n)):
        if n[i] <= 0:
            dbm[i] = -np.inf
        else:
            dbm[i] = 10*np.log10(1000*n[i])
    return dbm
def dBm_to_W(n):
    W = np.zeros(len(n))
    for i in range(len(n)):
        W[i] = 10**(n[i]/10)/1000
    return W
#P_lst = np.arange(0, 10, 0.01)
#f_lst = [390*10**6, 2.1*10**9, 7.2*10**9, 14.5*10**9]
#f_band = ['UHF', 'S-band', 'X-band', 'Ku-band']
f_lst = [390*10**6, 435*10**6, 906*10**6, 2025*10**6, 2200*10**6]
#f_lst = [435*10**6, 390*10**6, 906*10**6]
wl_lst = [3*10**8/ f for f in f_lst]
d_max = [43/(f/10**6)**(1/3) for f in f_lst]
#print(d_max)
R_array = np.zeros([len(wl_lst),len(P_lst)])
plt.clf()
fig, ax = plt.subplots()
for i in range(len(wl_lst)):
    for j in range(len(P_lst)):
        R_array[i,j] = wl_lst[i]/(4*np.pi) * np.sqrt((P_lst[j]*Gt*(Gr/Ts))/(CN_min*Kb))
#print(R_array)
#range = 4*np.pi/lambd * np.sqrt((P_lst*Gt*(Gr/Ts))/(CN_min*Kb)
for i in range(len(wl_lst)): ax.plot(dBm_lst, R_array[i,:], label=f"f = {f_lst[i]/10**6} MHz")
#plt.plot(dBm_lst,R*np.ones(len(dBm_lst)), label='Range = 20km')
#plt.title('Range as function of Power. C/N0='+str(round(CN_min,-5)))
plt.ylim(0,100)
plt.xlim(right=25)
#plt.yticks(np.arange(0,101,10))
plt.xlabel('Transmitter Power [dBm]')
plt.ylabel('range [km]')
plt.legend(loc=2)
# Create secondary x-axis (dBm)
ax2 = ax.twiny()
#ax2.spines['bot'].set_position(('axes', -0.1))
ax2.set_xlabel('Transmitter Power [W]')

# Synchronize the secondary x-axis with primary x-axis
dbm_ticks = np.array(ax.get_xticks())
watt_ticks = dBm_to_W(dbm_ticks)
watt_ticks = watt_ticks[np.isfinite(watt_ticks)]  # Remove any -inf values
ax2.set_xticks(dbm_ticks)
ax2.set_xticklabels([f'{w:.3g}' for w in watt_ticks])
ax2.set_xlim(ax.get_xlim())

#plt.savefig('Figures/P_R_2')
plt.show()