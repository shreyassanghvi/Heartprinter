# compute.py
import numpy as np
import struct
from dataclasses import dataclass

from pyqtgraph.opengl import MeshData


@dataclass
class MotorCommand:
    target_x: float
    target_y: float
    target_z: float
    execute: bool
    exit: bool

@dataclass
class HeartprinterStatus:
    current_baseL: [float, float, float]
    current_baseC: [float, float, float]
    current_baseR: [float, float, float]
    current_baseMP: [float, float, float]
    status: str

MOTOR_STRUCT_FORMAT = 'ddd??5x'
MOTOR_STRUCT_SIZE = struct.calcsize(MOTOR_STRUCT_FORMAT)

STATUS_STRUCT_FORMAT = 'dddddddddddd6s2x'
STATUS_STRUCT_SIZE = struct.calcsize(STATUS_STRUCT_FORMAT)

def read_all_points(filename):
    points = []
    with open(filename, 'r') as f:
        for i, line in enumerate(f):
            if i >= 4:
                break
            line = line.replace(',', ' ')
            tokens = line.strip().split()
            if len(tokens) >= 3:
                try:
                    p = [float(tokens[0]), float(tokens[1]), float(tokens[2])]
                    points.append(p)
                except ValueError:
                    continue
    if len(points) == 4:
        return np.array(points)
    return None

def compute_plane_mesh(points):
    p0, p1, p2 = points[0], points[1], points[2]
    v1 = p1 - p0
    v2 = p2 - p0
    normal = np.cross(v1, v2)
    norm_val = np.linalg.norm(normal)
    if norm_val > 1e-12:
        normal /= norm_val
    else:
        # Handle degenerate case, e.g., return zero vector or raise error
        normal = np.zeros_like(normal)

    n_grid = 15
    vertices = []
    faces = []

    for i in range(n_grid + 1):
        for j in range(n_grid +1 - i):
            u = i / n_grid
            v = j / n_grid
            point = p0 + u * v1 + v * v2
            vertices.append(point)

    vertices = np.array(vertices)
    vertex_idx = 0
    for i in range(n_grid + 1):
        for j in range(n_grid +1 - i):
            if i < n_grid and j < n_grid - i:
                curr_row_start = vertex_idx
                next_row_start = curr_row_start + (n_grid +1 - i)

                faces.append([
                    curr_row_start,
                    curr_row_start + 1,
                    next_row_start
                ])

                if j < n_grid - i - 1:
                    faces.append([
                        curr_row_start + 1,
                        next_row_start + 1,
                        next_row_start
                    ])
            vertex_idx += 1

    faces = np.array(faces)
    meshdata = MeshData(vertexes=vertices, faces=faces)
    return meshdata, normal  # return raw data, building GLMeshItem is UI's job

def triangle_area(p1, p2, p3):
    v1 = p2 - p1
    v2 = p3 - p1
    cross_product = np.cross(v1, v2)
    return 0.5 * np.linalg.norm(cross_product)

def validate_position(point, points):
    if points is None or len(points) < 3:
        return False

    a, b, c = points[0], points[1], points[2]
    ab = b - a
    ac = c - a
    normal = np.cross(ab, ac)
    normal_length = np.linalg.norm(normal)
    if normal_length < 1e-6:
        return False
    normal = normal / normal_length

    ap = point - a
    distance_to_plane = np.dot(ap, normal)
    proj_point = point - distance_to_plane * normal

    area_abc = triangle_area(a, b, c)
    area_pab = triangle_area(proj_point, a, b)
    area_pbc = triangle_area(proj_point, b, c)
    area_pca = triangle_area(proj_point, c, a)
    sum_sub_areas = area_pab + area_pbc + area_pca
    return abs(sum_sub_areas - area_abc) <= 1e-6

def compute_heart_frame_transform(points):
    centroid = np.mean(points, axis=0)
    y_axis = points[1] - centroid
    y_axis /= np.linalg.norm(y_axis)
    v = points[0] - centroid
    v_proj = v - np.dot(v, y_axis) * y_axis
    x_axis = v_proj / np.linalg.norm(v_proj)
    z_axis = np.cross(x_axis, y_axis)
    R = np.column_stack((x_axis, y_axis, z_axis))
    return R, centroid
