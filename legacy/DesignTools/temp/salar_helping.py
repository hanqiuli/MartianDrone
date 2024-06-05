import numpy as np
import matplotlib.pyplot as plt


if __name__=="__main__":
    alphas = np.linspace(0, 50, 1000)

    fuckery = np.linspace(0.1, 2, 1000)**2

    SalarSucksAss = (40*alphas[None, :]-0.5*alphas[None, :]**2+1e3) / fuckery[:, None]

    alphas_new = np.linspace(-50, 50, 1999)
    rootAss = SalarSucksAss[:, 0]
    newAss =  2* rootAss[:, None] - SalarSucksAss[:,::-1]

    plt.plot(alphas, SalarSucksAss[0, :])
    plt.plot(alphas, newAss[0, :])
    plt.plot(alphas, rootAss)
    plt.show()
    scipy.interp1d(rootAss)