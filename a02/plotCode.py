
import matplotlib.pyplot as plt

x = [512, 1024, 2048, 4096, 8192]
y_loop = [0.16530512614699547, 0.6754384569489048 , 2.6515777687498483, 10.658797777848667, 49.18732765315072]
y_np = [7.325799815589562e-05, 0.00044489890278782697, 0.0011644047481240705, 0.005764515753253363, 0.025250486850563903]

fig = plt.figure(figsize=(8, 6))

plt.plot(x, y_loop,label='loop')
plt.plot(x, y_np,label='numpy')
plt.xscale("log", basex=2)
plt.yscale("linear")
plt.legend()
plt.xlabel("2^n",fontsize=12)
plt.ylabel("t(s)",fontsize=12)
plt.title("MacOS, 2.3 GHz Dual-Core Intel Core i5, 16 GB",fontsize=12)
plt.show()