import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("mod.dat", delimiter=",")
if data.ndim == 1:
    data = data.reshape(1, -1)

if data.shape[1] == 2:
    i_vals = data[:, 0]
    q_vals = data[:, 1]
    title = "Constellation from mod.dat (I,Q)"
elif data.shape[1] >= 3:
    # format: col, I, Q
    i_vals = data[:, 1]
    q_vals = data[:, 2]
    title = "Constellation from mod.dat (col,I,Q)"
else:
    raise ValueError(f"Unsupported mod.dat format with {data.shape[1]} column(s)")

plt.figure(figsize=(6, 6))
plt.scatter(i_vals, q_vals, s=8, alpha=0.75)
plt.gca().set_aspect('equal', 'box')
plt.grid(True, alpha=0.3)
plt.xlabel("I")
plt.ylabel("Q")
plt.title(title)
plt.tight_layout()
plt.savefig("mod_constellation.png", dpi=150)

plt.show()
print("saved: mod_constellation.png")
