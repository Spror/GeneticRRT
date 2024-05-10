# from mpl_toolkits.mplot3d import Axes3D
# import numpy
# import matplotlib.pyplot as plt
# data = numpy.loadtxt('/home/wiktor/cxx/Sampling-basedMP/build/plik.txt')
# fig = plt.figure()
# plt.plot(data[:,1],data[:,2],'.-')
# plt.show()


from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt
data = numpy.loadtxt('/home/wiktor/cxx/Sampling-basedMP/build/plik.txt')
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(data[:,1],data[:,2],data[:,3],'.-')
plt.show()