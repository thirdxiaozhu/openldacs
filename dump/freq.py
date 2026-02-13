import numpy as np
import matplotlib.pyplot as plt

d = np.loadtxt("mod.dat", delimiter=",")  # col, I, Q
col = d[:, 0].astype(int)
I = d[:, 1]
Q = d[:, 2]

# 设为 None 画全部；改成整数可只看某一列，比如 0
sel_col = None

if sel_col is None:
    x, y = I, Q
    title = "cmat Constellation (all columns)"
else:
    m = (col == sel_col)
    x, y = I[m], Q[m]
    title = f"cmat Constellation (col={sel_col})"

plt.figure(figsize=(6,6))
plt.scatter(x, y, s=8, alpha=0.75)
plt.gca().set_aspect('equal', 'box')
plt.grid(True, alpha=0.3)
plt.xlabel("I")
plt.ylabel("Q")
plt.title(title)
plt.tight_layout()
plt.savefig("cmat_constellation.png", dpi=150)
plt.show()
