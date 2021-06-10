import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt

mean_position_error_all = genfromtxt('MPE_pf_d_20.csv', delimiter=',')
ff = [1/64, 1/16, 1/4, 4, 16, 64]
plt.figure()
for i in range(6):
    plt.plot(np.log2(ff[i] * np.ones(10)), mean_position_error_all[i,:], '.')
plt.xlabel('log2 scale of r')
plt.ylabel('Mean position error')
plt.show()

anees_all = genfromtxt('ANEES_pf_d_20.csv', delimiter=',')
ff = [1/64, 1/16, 1/4, 4, 16, 64]
plt.figure()
for i in range(6):
    plt.plot(np.log2(ff[i] * np.ones(10)), anees_all[i,:], '.')
plt.xlabel('log2 scale of r')
plt.ylabel('ANEES')
plt.show()