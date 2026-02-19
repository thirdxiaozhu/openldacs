import csv
import matplotlib.pyplot as plt

# x = []
# y = []
# with open("/home/jiaxv/ldacs/openldacs/cmake-build-debug/example/dump/corr_peak.csv", "r") as f:
#     for row in csv.reader(f):
#         x.append(int(row[0]))
#         y.append(float(row[1]))
#
# plt.plot(x, y)
# plt.title("Correlation Peak")
# plt.xlabel("Index")
# plt.ylabel("abs(P)")
# plt.savefig("corr_peak.png", dpi=150)
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

data = np.loadtxt("/home/jiaxv/ldacs/openldacs/cmake-build-debug/example/dump/corr_peak.csv", delimiter=",")
x = data[:, 0]
y = data[:, 1]

# 简单峰值检测（不依赖 scipy）
thr = 0.5 * y.max()
mask = (y[1:-1] > y[:-2]) & (y[1:-1] >= y[2:]) & (y[1:-1] > thr)
peak_x = x[1:-1][mask]
peak_y = y[1:-1][mask]

plt.figure(figsize=(12, 4))
plt.plot(x, y, lw=1.0, label="|M1|")
plt.scatter(peak_x, peak_y, s=18, c="r", label="peaks")
plt.axhline(thr, color="orange", ls="--", lw=1, label=f"threshold={thr:.3f}")
plt.xlabel("sample index")
plt.ylabel("magnitude")
plt.title("Correlation Peak")
plt.grid(alpha=0.3)
plt.legend()
plt.tight_layout()
plt.show()
# plt.savefig("dump/corr_peak.png", dpi=160)
print("saved: dump/corr_peak.png")
