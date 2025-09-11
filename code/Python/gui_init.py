import sys
import os
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QLabel,
    QGroupBox, QPushButton, QLineEdit, QMessageBox,
)
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData, GLMeshItem
from multiprocessing import shared_memory


def read_all_points(filename):
    points = []
    if os.path.exists(filename):
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
                        pass
    if len(points) == 4:
        return np.array(points)
    else:
        return None


def compute_plane_mesh(points):
    p0, p1, p2 = points[0], points[1], points[2]
    v1 = p1 - p0
    v2 = p2 - p0
    normal = np.cross(v1, v2)
    normal /= np.linalg.norm(normal)
    v1_norm = v1 / np.linalg.norm(v1)
    v2_proj = v2 - np.dot(v2, v1_norm) * v1_norm
    v2_norm = v2_proj / np.linalg.norm(v2_proj)
    n_grid = 20
    s1 = np.linspace(0, np.linalg.norm(v1) * 1.1, n_grid)
    s2 = np.linspace(0, np.linalg.norm(v2_proj) * 1.1, n_grid)
    S1, S2 = np.meshgrid(s1, s2)
    vertices = p0 + np.outer(S1.flatten(), v1_norm) + np.outer(S2.flatten(), v2_norm)
    vertices = vertices.reshape(-1, 3)
    faces = []
    for i in range(n_grid - 1):
        for j in range(n_grid - 1):
            idx = i * n_grid + j
            faces.append([idx, idx + 1, idx + n_grid])
            faces.append([idx + 1, idx + 1 + n_grid, idx + n_grid])
    faces = np.array(faces)
    meshdata = MeshData(vertexes=vertices, faces=faces)
    return meshdata, normal


def show_error_and_exit(self, message):
    dlg = QMessageBox(self)
    dlg.setWindowTitle("Error")
    dlg.setText(message)
    dlg.setIcon(QMessageBox.Critical)
    dlg.setStandardButtons(QMessageBox.Ok)
    dlg.buttonClicked.connect(lambda _: sys.exit(1))
    dlg.exec_()


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Plane & Points Visualization with Joystick & Input")
        self.resize(1280, 720)

        main_layout = QHBoxLayout(self)

        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=10, azimuth=270)
        main_layout.addWidget(self.view, stretch=2)

        right_panel = QVBoxLayout()

        # Coordinates display group
        right_group = QGroupBox("Points Coordinates")
        self.right_layout = QVBoxLayout()
        self.point_labels_3green = []
        for i in range(3):
            lbl = QLabel("")
            lbl.setAlignment(Qt.AlignCenter)
            self.point_labels_3green.append(lbl)
            self.right_layout.addWidget(lbl)

        right_group.setLayout(self.right_layout)
        right_panel.addWidget(right_group)

        # Jog Controls Group (arrows + WASD + Z + / -)
        jog_group = QGroupBox("Jog Controls (Joystick & WASD & Z +/-)")
        jog_layout = QGridLayout()
        btn_up = QPushButton("w")
        btn_up.clicked.connect(lambda: self.jog_axis(1, 1))  # Y +
        btn_left = QPushButton("a")
        btn_left.clicked.connect(lambda: self.jog_axis(0, -1))  # X -
        btn_right = QPushButton("s")
        btn_right.clicked.connect(lambda: self.jog_axis(2, 1))  # X +
        btn_down = QPushButton("d")
        btn_down.clicked.connect(lambda: self.jog_axis(1, -1))  # Y -
        home_button = QPushButton("Home")
        home_button.clicked.connect(self.home_to_centroid)

        btn_z_plus = QPushButton("+")
        btn_z_plus.clicked.connect(lambda: self.jog_axis(2, 1))  # Z +
        btn_z_minus = QPushButton("−")
        btn_z_minus.clicked.connect(lambda: self.jog_axis(2, -1))  # Z -

        jog_layout.addWidget(btn_up, 0, 1)
        jog_layout.addWidget(btn_left, 1, 0)
        jog_layout.addWidget(btn_right, 1, 2)
        jog_layout.addWidget(btn_down, 1, 1)
        jog_layout.addWidget(home_button, 2, 1)
        jog_layout.addWidget(btn_z_plus, 2, 0)
        jog_layout.addWidget(btn_z_minus, 2, 2)

        jog_group.setLayout(jog_layout)
        right_panel.addWidget(jog_group)

        # Input fields for X, Y, Z target coordinates with current position label in red between label and input
        input_group = QGroupBox("Target Position Input")
        input_layout = QGridLayout()
        self.inputs = {}
        self.current_pos_labels = {}

        for i, axis in enumerate(['X', 'Y', 'Z']):
            axis_label = QLabel(axis)
            axis_label.setAlignment(Qt.AlignCenter)
            axis_label.setFixedWidth(10)

            curr_label = QLabel("0.000")
            curr_label.setAlignment(Qt.AlignCenter)
            palette = curr_label.palette()
            palette.setColor(QPalette.WindowText, QColor('red'))
            curr_label.setPalette(palette)
            font = curr_label.font()
            font.setBold(True)
            curr_label.setFont(font)

            arrow_label = QLabel("←")
            arrow_label.setAlignment(Qt.AlignCenter)
            arrow_font = QFont()
            arrow_font.setPointSize(14)
            arrow_font.setBold(True)
            arrow_label.setFont(arrow_font)

            edit = QLineEdit()
            edit.setFixedWidth(50)
            edit.setAlignment(Qt.AlignCenter)
            edit.editingFinished.connect(
                lambda ax=axis, widget=edit: self.line_edit_changed(ax, widget)
            )

            input_layout.addWidget(axis_label, i, 0)
            input_layout.addWidget(curr_label, i, 1)
            input_layout.addWidget(arrow_label, i, 2)
            input_layout.addWidget(edit, i, 3)

            self.current_pos_labels[axis] = curr_label
            self.inputs[axis] = edit

        input_group.setLayout(input_layout)
        right_panel.addWidget(input_group)

        buttons_layout = QHBoxLayout()
        self.move_button = QPushButton("Move to Position")
        self.move_button.clicked.connect(self.move_to_position)
        buttons_layout.addWidget(self.move_button)
        right_panel.addLayout(buttons_layout)

        # Status Label for shared memory status
        self.status_label = QLabel("Status: Unknown")
        self.status_label.setAlignment(Qt.AlignCenter)
        right_panel.addWidget(self.status_label)

        right_panel.addStretch()
        main_layout.addLayout(right_panel, stretch=0)

        self.setLayout(main_layout)

        # Load points
        self.current_pos = None
        self.points = None
        self.target_pos = None

        pts = read_all_points('input.txt')
        if pts is None or len(pts) < 4:
            show_error_and_exit(self, "Input file must contain at least 4 valid points.")
        else:
            self.points = pts[:3]
            self.fourth_point = pts[3]
        labels = ["Left", "Center", "Right"]

        coord_grid_layout = QGridLayout()
        for i, p in enumerate(self.points):
            lbl_name = QLabel(labels[i])
            lbl_name.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

            lbl_colon = QLabel(":")
            lbl_colon.setAlignment(Qt.AlignCenter)
            lbl_colon.setFixedWidth(10)
            colon_font = QFont()
            colon_font.setBold(True)
            lbl_colon.setFont(colon_font)

            lbl_coord = QLabel(f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
            lbl_coord.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

            coord_grid_layout.addWidget(lbl_name, i, 0)
            coord_grid_layout.addWidget(lbl_colon, i, 1)
            coord_grid_layout.addWidget(lbl_coord, i, 2)

        for i in reversed(range(self.right_layout.count())):
            self.right_layout.itemAt(i).widget().setParent(None)

        wrapper_widget = QWidget()
        wrapper_widget.setLayout(coord_grid_layout)
        self.right_layout.addWidget(wrapper_widget)

        self.current_pos = self.fourth_point.copy()
        self.target_pos = np.mean(self.points, axis=0)

        self.scatter_green = gl.GLScatterPlotItem(
            pos=self.points, size=10, color=(0, 1, 0, 1), pxMode=True)
        self.view.addItem(self.scatter_green)

        self.scatter_red = gl.GLScatterPlotItem(
            pos=np.array([self.current_pos]), size=15, color=(1, 0, 0, 1), pxMode=True)
        self.view.addItem(self.scatter_red)

        self.scatter_blue = gl.GLScatterPlotItem(
            pos=np.array([self.target_pos]), size=15, color=(0, 0, 1, 1), pxMode=True)
        self.view.addItem(self.scatter_blue)

        meshdata, normal = compute_plane_mesh(self.points)
        self.plane_mesh = GLMeshItem(
            meshdata=meshdata,
            smooth=False,
            color=(0.5, 0.7, 1.0, 0.15),
            shader='normalColor',
            drawEdges=True,
            edgeColor=(0, 0, 0, 0.8),
            glOptions='additive'
        )
        self.view.addItem(self.plane_mesh)

        self.update_inputs_from_target()
        self.update_labels()

        # For frame limiting: flag to track data changes
        self._data_changed = True

        # Open existing C++ created shared memory (read-only from Python perspective)
        try:
            self.read_shm = shared_memory.SharedMemory(name="Local\\CPPToPy")
        except FileNotFoundError:
            print("C++ -> Python shared memory not found")
            self.read_shm = None

        if self.read_shm:
            self.read_timer = QTimer(self)
            self.read_timer.timeout.connect(self.read_from_cpp)
            self.read_timer.start(500)  # 500ms poll

        # Create Python -> C++ shared memory for writing
        try:
            self.write_shm = shared_memory.SharedMemory(name="Local\\PyToCPP", create=True, size=1024)
            self.write_shm.buf[:] = b'\0' * 1024
        except FileExistsError:
            self.write_shm = shared_memory.SharedMemory(name="Local\\PyToCPP")


        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status_from_shared_memory)
        self.status_timer.start(500)  # status update every 500 ms

        # Timer for redraw with frame limiting (~30 FPS)
        self.redraw_timer = QTimer(self)
        self.redraw_timer.timeout.connect(self.redraw_if_needed)
        self.redraw_timer.start(33)  # ~30 FPS

    def redraw_if_needed(self):
        if self._data_changed:
            # Update 3D scatter plots position
            self.scatter_red.setData(pos=np.array([self.current_pos]))
            self.scatter_blue.setData(pos=np.array([self.target_pos]))
            # Could update other items similarly if dynamic

            self.view.update()  # Request redraw
            self._data_changed = False

    def write_to_cpp(self):
        if not self.write_shm:
            return
        try:
            # Example data: send target position
            data_str = f"{self.target_pos[0]:.3f},{self.target_pos[1]:.3f},{self.target_pos[2]:.3f}"
            data_bytes = data_str.encode('utf-8')[:1023] + b'\0'
            self.write_shm.buf[:len(data_bytes)] = data_bytes
        except Exception as e:
            print(f"Write error: {e}")

    def update_status_from_shared_memory(self):
        try:
            buf = self.shared_mem.buf[:]  # or larger if needed

            # Convert bytes to string by extracting up to null terminator
            message_bytes = bytes(buf).split(b'\0', 1)[0]
            message_str = message_bytes.decode('utf-8', errors='ignore')

            self.status_label.setText(f"Status: {message_str}")
        except Exception:
            self.status_label.setText("Status: Read error")

    def closeEvent(self, event):
        if self.read_shm:
            self.read_shm.close()
            self.read_shm = None
        if self.write_shm:
            self.write_shm.close()
            try:
                self.write_shm.unlink()
            except FileNotFoundError:
                pass
            self.write_shm = None
        event.accept()

    def home_to_centroid(self):
        if self.points is not None:
            self.target_pos = np.mean(self.points, axis=0)
            self._data_changed = True
            self.update_inputs_from_target()
            self.update_labels()
            self.move_to_position()

    def move_to_position(self):
        if self.target_pos is not None:
            self.write_to_cpp()
            self.current_pos = self.target_pos.copy()
            self._data_changed = True
            self.update_labels()

    def line_edit_changed(self, axis, widget):
        try:
            val = float(widget.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", f"Please enter a valid number for {axis}")
            self.update_inputs_from_target()
            return
        idx = {'X': 0, 'Y': 1, 'Z': 2}[axis]
        self.target_pos[idx] = val
        self._data_changed = True
        self.update_labels()

    def update_inputs_from_target(self):
        for axis in ['X', 'Y', 'Z']:
            idx = {'X': 0, 'Y': 1, 'Z': 2}[axis]
            val = self.target_pos[idx]
            self.inputs[axis].blockSignals(True)
            self.inputs[axis].setText(f"{val:.3f}")
            self.inputs[axis].blockSignals(False)

    def jog_axis(self, axis, direction):
        if self.target_pos is not None:
            self.target_pos[axis] += direction * 0.1
            self._data_changed = True
            self.update_inputs_from_target()
            self.update_labels()

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_W:
            self.jog_axis(1, 1)
        elif key == Qt.Key_S:
            self.jog_axis(1, -1)
        elif key == Qt.Key_A:
            self.jog_axis(0, -1)
        elif key == Qt.Key_D:
            self.jog_axis(0, 1)
        elif key == Qt.Key_Plus:
            self.jog_axis(2, 1)
        elif key == Qt.Key_Minus:
            self.jog_axis(2, -1)
        elif key == Qt.Key_Enter or key == Qt.Key_Return:
            self.move_to_position()
        elif key == Qt.Key_Z:
            self.home_to_centroid()
        else:
            super().keyPressEvent(event)

    def update_labels(self):
        if self.current_pos is not None:
            self.current_pos_labels['X'].setText(f"{self.current_pos[0]:.3f}")
            self.current_pos_labels['Y'].setText(f"{self.current_pos[1]:.3f}")
            self.current_pos_labels['Z'].setText(f"{self.current_pos[2]:.3f}")
        else:
            self.current_pos_labels['X'].setText("N/A")
            self.current_pos_labels['Y'].setText("N/A")
            self.current_pos_labels['Z'].setText("N/A")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
