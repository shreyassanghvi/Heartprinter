import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import numpy as np

point4_history = []

def read_dof_file(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    data = [list(map(float, line.strip().split(','))) for line in lines]
    return data

def compute_plane(p1, p2, p3):
    v1 = np.array(p2) - np.array(p1)
    v2 = np.array(p3) - np.array(p1)
    normal = np.cross(v1, v2)
    d = -np.dot(normal, p1)
    return normal, d

def point_plane_distance(normal, d, point):
    num = abs(np.dot(normal, point) + d)
    denom = np.linalg.norm(normal)
    return num / denom

def euler_to_matrix(roll, pitch, yaw):
    r, p, y = np.deg2rad([roll, pitch, yaw])
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    return R

def plot_orientation(ax, origin, roll, pitch, yaw, length=1.0):
    R = euler_to_matrix(roll, pitch, yaw)
    axes = np.eye(3) * length
    colors = ['r', 'g', 'b']
    for i in range(3):
        vec = R @ axes[:, i]
        ax.quiver(*origin, *vec, color=colors[i])

def plot_plane(ax, normal, d, points):
    xx, yy = np.meshgrid(
        np.linspace(min([p[0] for p in points])-1, max([p[0] for p in points])+1, 10),
        np.linspace(min([p[1] for p in points])-1, max([p[1] for p in points])+1, 10)
    )
    a, b, c = normal
    if c != 0:
        zz = (-a * xx - b * yy - d) / c
        ax.plot_surface(xx, yy, zz, alpha=0.3, color='cyan')

def update_plot():
    try:
        data = read_dof_file('input.txt')
        p1, p2, p3, p4 = [np.array(d[:3]) for d in data]
        roll, pitch, yaw = data[3][3:6]
        normal, d = compute_plane(p1, p2, p3)
        dist = point_plane_distance(normal, d, p4)

        # Add current 4th point to history
        point4_history.append(p4.copy())

        ax.clear()
        ax.scatter(*zip(*[p1, p2, p3]), color='blue', label='Reference Points')
        ax.scatter(*p4, color='red', label='Current 4th Point')
        plot_plane(ax, normal, d, [p1, p2, p3, p4])
        plot_orientation(ax, p4, roll, pitch, yaw, length=1.0)
        ax.text(*p4, f'Dist: {dist:.2f}', color='red')

        # Dynamic color grading for point cloud based on Z (height)
        if point4_history:
            z_vals = np.array([pt[2] for pt in point4_history])
            z_min, z_max = np.min(z_vals), np.max(z_vals)
            if z_max > z_min:
                norm = (z_vals - z_min) / (z_max - z_min)
            else:
                norm = np.full_like(z_vals, 0.5)
            cmap = plt.get_cmap('viridis')
            colors = cmap(norm)
            xs, ys, zs = zip(*point4_history)
            ax.scatter(xs, ys, zs, color=colors, alpha=0.7, label='4th Point History (Height Color)')

        ax.legend()
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title("3D Reference Plane, 4th Point, Orientation, and Dynamically Normalized Point Cloud")
        canvas.draw()
    except Exception as e:
        pass
    root.after(1000, update_plot)

root = tk.Tk()
root.title("3D Plane Visualization with Dynamic Color-Graded Point Cloud")

fig = plt.figure(figsize=(20, 20))
ax = fig.add_subplot(111, projection='3d')
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

update_plot()
root.mainloop()
