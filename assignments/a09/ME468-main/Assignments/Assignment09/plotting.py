
import matplotlib.pyplot as plt

x = [0.1, 0.01, 0.001, 0.0001, 0.00001]
#forward
error_position = [1806.118729581586, 0.0331835666943265, 0.003164229412425612, 0.00031499011418507916, 3.1484789554045434e-05]
error_velocity = [21691.71354861062, 0.9564903562626734, 0.1040665019940078, 0.010490666503240131, 0.001049906667196605]
#backward
b_error_position = [0.26923076923076916, 0.02755759564831689, 0.0028066258641096797, 0.00028114648640398254, 2.8119422754990797e-05]
b_error_velocity = [2.846827870687055, 0.7492870268928757, 0.1018423137306313, 0.010468282196653035, 0.0010496826828779488]


fig = plt.figure(figsize=(8, 6))

plt.plot(x, b_error_position,label='position')
plt.scatter(x, b_error_position)
plt.plot(x, b_error_velocity,label='velocity')
plt.scatter(x, b_error_velocity)

plt.xscale("log")
plt.yscale("log")
plt.legend()
plt.xlabel("timestep",fontsize=12)
plt.ylabel("error",fontsize=12)
plt.title("Backward Euler: Error of position 1E-1 to 1E-5",fontsize=12)
#plt.title("Forward Euler: Error of position 1E-1 to 1E-5",fontsize=12)
plt.show()