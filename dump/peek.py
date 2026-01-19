import csv
import matplotlib.pyplot as plt

x = []
y = []
with open("/home/jiaxv/ldacs/openldacs/cmake-build-debug/example/dump/corr_peak.csv", "r") as f:
    for row in csv.reader(f):
        x.append(int(row[0]))
        y.append(float(row[1]))

plt.plot(x, y)
plt.title("Correlation Peak")
plt.xlabel("Index")
plt.ylabel("abs(P)")
plt.savefig("corr_peak.png", dpi=150)
