import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def load_data(file_path):
    # Wczytanie danych z pliku, pomijając trzy ostatnie kolumny
    data = np.loadtxt(file_path)
    data = data[:, :-4]
    return data

def plot_sphere_with_data(data, center, radius):
    # Przygotowanie danych dla kuli
    phi, theta = np.mgrid[0.0:np.pi:100j, 0.0:2.0*np.pi:100j]
    x = center[0] + radius * np.sin(phi) * np.cos(theta)
    y = center[1] + radius * np.sin(phi) * np.sin(theta)
    z = center[2] + radius * np.cos(phi)

    # Tworzenie wykresu 3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Rysowanie kuli
    ax.plot_surface(x, y, z, color='b', alpha=0.3)

    # Rysowanie punktów z pliku
    if data.size > 0:
        ax.plot(data[:, 0], data[:, 1], data[:, 2], color='r')

    # Ustawienie zakresów osi
    ax.set_xlim([0, 500])
    ax.set_ylim([0, 500])
    ax.set_zlim([0, 500])

    # Etykiety osi
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Tytuł wykresu
    ax.set_title('Kula i punkty z pliku w układzie współrzędnych 3D')

    # Wyświetlenie wykresu
    plt.show()

# Środek kuli
center = np.array([250, 250, 250])
# Promień kuli
radius = 100

# Ścieżka do pliku (należy podać odpowiednią ścieżkę)
file_path = '/home/wiktor/cxx/Sampling-basedMP/build/plik.txt'

# Wczytanie danych z pliku
data = load_data(file_path)

# Rysowanie wykresu
plot_sphere_with_data(data, center, radius)
