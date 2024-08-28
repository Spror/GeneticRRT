import sys
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def help():
    print("#### HELP ####")
    print("######################################")
    print("Python program to visualize paths in the SE3 space. Use special arguments to call: ")
    print("SE3_1 -- Space without obstcles")
    print("SE3_2 -- Space with one obstacle")
    print("SE3_3 -- City simulation")

def read_data(file_path):
    
    with open(file_path, 'r') as file:
        lines = file.readlines()

    data = []
    current_data = []

    for line in lines:
        if line.strip() == '':
            if current_data:
                data.append(np.array(current_data)[:, :-4])
                current_data = []
        else:
            current_data.append(list(map(float, line.split())))

    if current_data:
        data.append(np.array(current_data)[:, :-4])

    return data

def draw_rectangular(ax, x, y, z, dx, dy, dz, color):
    #Define the vertices of a rectangle
    vertices = [
        [x, y, z],
        [x+dx, y, z],
        [x+dx, y+dy, z],
        [x, y+dy, z],
        [x, y, z+dz],
        [x+dx, y, z+dz],
        [x+dx, y+dy, z+dz],
        [x, y+dy, z+dz]
    ]
    # Define the walls of a rectangle
    faces = [
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[1], vertices[2], vertices[6], vertices[5]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[3], vertices[0], vertices[4], vertices[7]],
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]]
    ]
   
    poly3d = Poly3DCollection(faces, facecolors=color, alpha=0.5)
    ax.add_collection3d(poly3d)

def drawSE3_1(data):
    # draw SE3 #1 scenario - space without obstacles
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(10, 10, 10, 'ro', label='Start')  # Start point
    ax.plot(400, 400, 400, 'go', label='Goal')  # Goal point
    
    # draw paths from the file
    for i, data in enumerate(data):
        if data.size > 0:
                ax.plot(data[:, 0], data[:, 1], data[:, 2], label=str(i+1))

    # set axis limits
    ax.set_xlim([0, 500])
    ax.set_ylim([0, 500])
    ax.set_zlim([0, 500])

    # set axis labels
    ax.set_xlabel('X', fontsize = 16)
    ax.set_ylabel('Y', fontsize = 16)
    ax.set_zlabel('Z', fontsize = 16)

    # set title
    ax.set_title('#1', fontsize = 16)

    # set legend
    ax.legend(loc='center left', bbox_to_anchor=(1, 1.0))

    plt.show()

def drawSE3_2(data):
    # draw SE3 #2 scenario - space with one obstacle
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(10, 10, 10, 'ro', label='Start')  # Start point
    ax.plot(400, 400, 400, 'go', label='Goal')  # Goal point

    # ball data
    center = np.array([250, 250, 250])
    radius = 150

    phi, theta = np.mgrid[0.0:np.pi:100j, 0.0:2.0*np.pi:100j]
    x = center[0] + radius * np.sin(phi) * np.cos(theta)
    y = center[1] + radius * np.sin(phi) * np.sin(theta)
    z = center[2] + radius * np.cos(phi)

    ax.plot_surface(x, y, z, color='b', alpha=0.3)

    # draw paths from the file
    for i, data in enumerate(data):
        if data.size > 0:
                ax.plot(data[:, 0], data[:, 1], data[:, 2], label=str(i+1))

    # set axis limits
    ax.set_xlim([0, 500])
    ax.set_ylim([0, 500])
    ax.set_zlim([0, 500])

    # set axis labels
    ax.set_xlabel('X', fontsize = 16)
    ax.set_ylabel('Y', fontsize = 16)
    ax.set_zlabel('Z', fontsize = 16)

    # set title 
    ax.set_title('#2', fontsize = 16)

    # set legend
    ax.legend(loc='center left', bbox_to_anchor=(1, 1.0))

    plt.show()


def drawSE3_3(data):
    # draw SE3 #3 scenario - city simulation
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # define buildings as a rectangles
   
    draw_rectangular(ax, 150, 250, 0, 40, 40, 200, 'lime')
    draw_rectangular(ax, 80, 20, 0, 40, 40, 250, 'olive')
    draw_rectangular(ax, 20, 80, 0, 40, 40, 350, 'b')
    draw_rectangular(ax, 250, 250, 0, 60, 60, 400, 'brown')
    draw_rectangular(ax, 100, 100, 0, 30, 30, 150, 'y')
    draw_rectangular(ax, 150, 150, 0, 50, 50, 300, 'c')
    draw_rectangular(ax, 300, 100, 0, 40, 40, 200, 'm')
    draw_rectangular(ax, 350, 200, 0, 50, 50, 250, 'orange')
    draw_rectangular(ax, 200, 350, 0, 70, 70, 100, 'purple')
    draw_rectangular(ax, 400, 100, 0, 50, 50, 150, 'lime')
    draw_rectangular(ax, 50, 400, 0, 50, 50, 200, 'cyan')
    draw_rectangular(ax, 100, 300, 0, 40, 40, 250, 'magenta')
    draw_rectangular(ax, 250, 100, 0, 60, 60, 300, 'salmon')
    draw_rectangular(ax, 150, 350, 0, 70, 70, 350, 'skyblue')
    draw_rectangular(ax, 50, 50, 0, 30, 30, 120, 'khaki')
    draw_rectangular(ax, 300, 300, 0, 50, 50, 250, 'orchid')
    draw_rectangular(ax, 350, 50, 0, 40, 40, 180, 'gold')
    draw_rectangular(ax, 150, 50, 0, 50, 50, 220, 'olive')
    draw_rectangular(ax, 400, 400, 0, 60, 60, 270, 'brown')

    ax.plot(10, 10, 10, 'ro', label='Start')  # Start point
    ax.plot(490, 490, 100, 'go', label='Goal')  # Goal point


    # draw paths from the file
    for i, data in enumerate(data):
        if data.size > 0:
                ax.plot(data[:, 0], data[:, 1], data[:, 2], label=str(i+1))

    # set axis limits
    ax.set_xlim([0, 500])
    ax.set_ylim([0, 500])
    ax.set_zlim([0, 500])

    # set axis labels
    ax.set_xlabel('X', fontsize = 16)
    ax.set_ylabel('Y', fontsize = 16)
    ax.set_zlabel('Z', fontsize = 16)

    # set title
    ax.set_title('#3', fontsize = 16)

    # set legend
    ax.legend(loc='center left', bbox_to_anchor=(1, 1.0))

    plt.show()

if __name__ == '__main__':

    # Read data from file
    filename = '/home/wiktor/cxx/Sampling-basedMP/build/plik.txt'
    data = read_data(filename)

    if len(sys.argv) == 2:
        if sys.argv[1] == 'SE3_1':
             drawSE3_1(data)
        elif sys.argv[1] == 'SE3_2':
             drawSE3_2(data)
        elif sys.argv[1] == 'SE3_3':
             drawSE3_3(data)
        else:
            help()
    
    else:   
        help()
