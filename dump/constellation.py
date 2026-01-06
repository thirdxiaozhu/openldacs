import numpy as np
import matplotlib.pyplot as plt

d = np.loadtxt("mod.dat")
plt.scatter(d[:,0], d[:,1], s=8)
plt.gca().set_aspect('equal', 'box')
plt.grid(True)
plt.xlabel("I")
plt.ylabel("Q")
plt.title("Constellation")
plt.show()