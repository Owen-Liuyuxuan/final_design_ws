import scipy.io as sio
import numpy as np

controller_dict = sio.loadmat('H_miu_control_matrix.mat')
controllerA = np.array(controller_dict['KA'], dtype = np.float64).copy()
controllerB = np.array(controller_dict['KB'], dtype = np.float64).copy()
controllerC = np.array(controller_dict['KC'], dtype = np.float64).copy()
controllerD = np.array(controller_dict['KD'], dtype = np.float64).copy()

np.save("KA.npy", controllerA)
np.save("KB.npy", controllerB)
np.save("KC.npy", controllerC)
np.save("KD.npy", controllerD)

print(controllerB.shape)

print controllerA
print controllerB
print controllerC
print controllerD
