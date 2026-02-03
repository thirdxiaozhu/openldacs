import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
df = pd.read_csv("/home/jiaxv/ldacs/openldacs/cmake-build-debug/example/time_domain_waveform.csv", header=None, names=["index", "real", "imag"])

# 绘制实部和虚部
plt.figure(figsize=(10, 6))
plt.plot(df["index"], df["real"], label="Real Part")
plt.plot(df["index"], df["imag"], label="Imaginary Part")
plt.xlabel("Sample Index")
plt.ylabel("Amplitude")
plt.title("Time Domain Waveform")
plt.legend()
plt.grid(True)
plt.show()