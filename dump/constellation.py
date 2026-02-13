import numpy as np
import matplotlib.pyplot as plt

d = np.loadtxt("mod.dat", delimiter=",")  # 两列: I,Q

plt.figure(figsize=(6,6))
plt.scatter(d[:,0], d[:,1], s=8, alpha=0.75)
plt.gca().set_aspect('equal', 'box')
plt.grid(True, alpha=0.3)
plt.xlabel("I")
plt.ylabel("Q")
plt.title("itpp::cvec Constellation")
plt.tight_layout()
plt.savefig("cvec_constellation.png", dpi=150)
plt.show()
