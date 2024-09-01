import sys
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon

def help():
    print("#### HELP ####")
    print("######################################")
    print("Python program to visualize paths in the SE3 space. Use special arguments to call: ")
    print("SE2_1 -- Space without obstcles")
    print("SE2_2 -- Space with one obstacle")
    print("SE2_3 -- Maze simulation")

# Define the shapes
def draw_circle(ax, center_x, center_y, radius):
    theta = np.linspace(0, 2 * np.pi, 100)
    x = center_x + radius * np.cos(theta)
    y = center_y + radius * np.sin(theta)
    ax.fill(x, y, 'b', alpha=0.5)  # Fill circle with blue color

def draw_rectangle(ax, lower_left_x, lower_left_y, width, height):
    rect = Rectangle((lower_left_x, lower_left_y), width, height, color='b', alpha=0.5)
    ax.add_patch(rect)

def draw_triangle(ax, vertices):
    triangle = Polygon(vertices, closed=True, color='b', alpha=0.5)
    ax.add_patch(triangle)

def draw_polygon(ax, vertices, color):
    polygon = Polygon(vertices, closed=True, color=color, alpha=0.5)
    ax.add_patch(polygon)

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

def drawSE2_1(data):
    fig, ax = plt.subplots()

    # Add points
    ax.plot(10, 10, 'ro', label='Start')  # Start point
    ax.plot(450, 440, 'go', label='Goal')  # Goal point

    # Plot curves from data (if available)
    for i, curve in enumerate(data):
        curve = np.array(curve)
        if i == 0:  # Only label the first three curves
            ax.plot(curve[:,0], curve[:,1], label="GeneticRRT")
        elif i == 1:
            ax.plot(curve[:,0], curve[:,1], label="RRT")
        elif i == 2:
            ax.plot(curve[:,0], curve[:,1], label="RRT*")
        else:
            ax.plot(curve[:,0], curve[:,1])

    # Set labels and title
    ax.set_xlabel('X',fontsize=16)
    ax.set_ylabel('Y',fontsize=16)
    ax.set_title('#1',fontsize=16)
    ax.legend(loc='center left', bbox_to_anchor=(1, 1.0))
    ax.set_xlim(0, 500)
    ax.set_ylim(0, 500)
    ax.set_aspect('equal')  # Keep the aspect ratio of the plot

    plt.show()


def drawSE2_2(data):

    # Create Circle 
    center_x = 250
    center_y = 250

    radius = 150

    theta = np.linspace(0, 2 * np.pi, 100)
    x = center_x + radius * np.cos(theta)
    y = center_y + radius * np.sin(theta)

    fig, ax = plt.subplots()
    ax.fill(x, y, 'b')  # Zamalowanie koła na niebiesko
    ax.set_aspect('equal')  # Utrzymanie proporcji osi

    # Dodanie punktów
    ax.plot(10, 10, 'ro' , label='Start')  # Punkt w (1, 1) zaznaczony na czerwono
    ax.plot(400, 400, 'go', label='Goal')  # Punkt w (400, 400) zaznaczony na zielono

    for i, curve in enumerate(data):
        curve = np.array(curve)
        if i == 0:  # Only label the first three curves
            ax.plot(curve[:,0], curve[:,1], label="GeneticRRT")
        elif i == 1:
            ax.plot(curve[:,0], curve[:,1], label="RRT")
        elif i == 2:
            ax.plot(curve[:,0], curve[:,1], label="RRT*")
        else:
            ax.plot(curve[:,0], curve[:,1])

    # Set labels and title
    ax.set_xlabel('X',fontsize=16)
    ax.set_ylabel('Y',fontsize=16)
    ax.set_title('#2',fontsize=16)
    ax.legend(loc='center left', bbox_to_anchor=(1, 1.0))
    ax.set_xlim(0, 500)
    ax.set_ylim(0, 500)
    ax.set_aspect('equal')  # Keep the aspect ratio of the plot
    plt.show()


def drawSE2_3(data):
    fig, ax = plt.subplots()

    # Draw shapes
    draw_rectangle(ax, lower_left_x=0, lower_left_y=50, width=200, height=20)
    draw_rectangle(ax, lower_left_x=200, lower_left_y=50, width=20, height=200)
    draw_rectangle(ax, lower_left_x=250, lower_left_y=250, width=50, height=50)
    draw_rectangle(ax, lower_left_x=400, lower_left_y=100, width=80, height=60)
    draw_rectangle(ax, lower_left_x=250, lower_left_y=150, width=40, height=70)
    draw_circle(ax, 120, 200, 50)
    draw_triangle(ax,[[400,400], [350,450], [500,500]])
    draw_polygon(ax, vertices=[[50, 400], [350, 320], [320, 360], [280, 350], [200, 490]], color='b')

    # Add points
    ax.plot(10, 10, 'ro', label='Start')  # Start point
    ax.plot(100, 100, 'go', label='Goal')  # Goal point

    # Plot curves from data (if available)
    for i, curve in enumerate(data):
        curve = np.array(curve)
        if i == 0:  # Only label the first three curves
            ax.plot(curve[:,0], curve[:,1], label="GeneticRRT")
        elif i == 1:
            ax.plot(curve[:,0], curve[:,1], label="RRT")
        elif i == 2:
            ax.plot(curve[:,0], curve[:,1], label="RRT*")
        else:
            ax.plot(curve[:,0], curve[:,1])

    # Set labels and title
    ax.set_xlabel('X',fontsize=16)
    ax.set_ylabel('Y',fontsize=16)
    ax.set_title('#3',fontsize=16)
    ax.legend(loc='center left', bbox_to_anchor=(1, 1.0))
    ax.set_xlim(0, 500)
    ax.set_ylim(0, 500)
    ax.set_aspect('equal')  # Keep the aspect ratio of the plot

    plt.show()

if __name__ == '__main__':

    # Read data from file
    filename = '/home/wiktor/cxx/Sampling-basedMP/build/file.txt'
    data = read_data(filename)

    if len(sys.argv) == 2:
        if sys.argv[1] == 'SE2_1':
             drawSE2_1(data)
        elif sys.argv[1] == 'SE2_2':
             drawSE2_2(data)
        elif sys.argv[1] == 'SE2_3':
             drawSE2_3(data)
        else: 
            help()
    
    else:
        help()
   