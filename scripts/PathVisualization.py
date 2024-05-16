from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
# data = numpy.loadtxt('/home/wiktor/cxx/Sampling-basedMP/build/plik.txt')
# fig = plt.figure()
# plt.plot(data[:,0],data[:,1],'.-')
# rec = Rectangle(xy = (100.0,100.0),width=150.0,height=150.0, angle= 0.0, color = 'r')
# fig = plt.gcf()
# ax= fig.gca()
# ax.add_patch(rec)
# plt.show()


# # from mpl_toolkits.mplot3d import Axes3D
# # import numpy
# # import matplotlib.pyplot as plt
# # data = numpy.loadtxt('/home/wiktor/cxx/Sampling-basedMP/build/plik.txt')
# # fig = plt.figure()
# # ax = fig.gca(projection='3d')
# # ax.plot(data[:,1],data[:,2],data[:,3],'.-')
# # plt.show()
import matplotlib.pyplot as plt

# Function to read data from file
def read_data(filename):
    with open(filename, 'r') as file:
        data = []
        curve = []
        for line in file:
            if line.strip() == "":
                if curve:
                    data.append(curve)
                    curve = []
            else:
                point = list(map(float, line.split()))
                curve.append(point)
        if curve:
            data.append(curve)
    return data

# Read data from file
filename = '/home/wiktor/cxx/Sampling-basedMP/build/plik.txt'
data = read_data(filename)

# Plotting the data
fig, ax = plt.subplots()

for curve in data:
    curve = np.array(curve)
    ax.plot(curve[:,0], curve[:,1])

# Set labels
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
rec = Rectangle(xy = (100.0,100.0),width=150.0,height=150.0, angle= 0.0, color = 'r')
fig = plt.gcf()
ax= fig.gca()
ax.add_patch(rec)
plt.show()
