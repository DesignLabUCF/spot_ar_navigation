import numpy as np
from matplotlib import pyplot as plt



arr = np.load('/home/sdl/Desktop/minerva-sensors/build/ProcessedImages/cap221030/npy/0-raw16_221030.npy')
plt.imshow(arr, interpolation='nearest')
plt.show()