import sys
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QLabel,
    QGroupBox, QPushButton, QLineEdit, QMessageBox,
)
from PyQt5.QtGui import QPalette, QColor, QFont, QVector3D
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLMeshItem, MeshData
from multiprocessing import shared_memory
import struct

from compute import (
    MOTOR_STRUCT_FORMAT, MOTOR_STRUCT_SIZE,
    STATUS_STRUCT_FORMAT, STATUS_STRUCT_SIZE,
    compute_plane_mesh, validate_position, compute_heart_frame_transform
)

class SyncableGLViewWidget(gl.GLViewWidget):
    cameraMoved = pyqtSignal(float, float)  # azimuth, elevation

    def __init__(self):
        super().__init__()
        self._last_azimuth = self.opts.get('azimuth', 0)
        self._last_elevation = self.opts.get('elevation', 0)

    def mouseMoveEvent(self, ev):
        super().mouseMoveEvent(ev)
        azim = self.opts.get('azimuth', 0)
        elev = self.opts.get('elevation', 0)
        # emit signal if camera orientation changes
        if azim != self._last_azimuth or elev != self._last_elevation:
            self._last_azimuth = azim
            self._last_elevation = elev
            self.cameraMoved.emit(azim, elev)

class OrientationAxes(gl.GLViewWidget):
    cameraChanged = pyqtSignal()  # Signal to notify camera changes

    def __init__(self, size=2.0):
        super().__init__()
        self.setFixedSize(80, 80)
        self.opts['distance'] = size
        self.opts['center'] = QVector3D(0, 0, 0)
        self.setCameraPosition(distance=size, elevation=30, azimuth=60)
        axes = [
            ([0, 0, 0], [1, 0, 0], (1, 0, 0, 1)),  # X Red
            ([0, 0, 0], [0, 1, 0], (0, 1, 0, 1)),  # Y Green
            ([0, 0, 0], [0, 0, 1], (0, 0, 1, 1)),  # Z Blue
        ]
        for start, end, color in axes:
            line = gl.GLLinePlotItem(
                pos=np.array([start, end], dtype=float),
                color=color, width=3, antialias=True, mode='lines'
            )
            self.addItem(line)

        self.dragging = False
        self.last_pos = None

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self.dragging = True
            self.last_pos = ev.pos()
            ev.accept()
        else:
            super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self.dragging:
            dx = ev.x() - self.last_pos.x()
            dy = ev.y() - self.last_pos.y()
            self.last_pos = ev.pos()

            # Update azimuth and elevation based on mouse movement
            azimuth = (self.opts['azimuth'] - dx) % 360
            elevation = np.clip(self.opts['elevation'] - dy, -90, 90)
            self.setCameraPosition(distance=self.opts['distance'], azimuth=azimuth, elevation=elevation)

            self.cameraChanged.emit()
            ev.accept()
        else:
            super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if ev.button() == Qt.LeftButton and self.dragging:
            self.dragging = False
            ev.accept()
        else:
            super().mouseReleaseEvent(ev)


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Plane & Points Visualization")
        self.resize(1280, 720)
        self.points = np.zeros((3, 3))
        self.current_pos = np.zeros(3)
        self.target_pos = np.zeros(3)
        self._data_changed = False

        self.main_layout = QHBoxLayout(self)
        self.setLayout(self.main_layout)

        self.init_left_panel()
        self.init_right_panel()
        self.init_shared_memory()
        self.init_visualization()
        self.init_action_buttons()
        self.init_orientation_axes()

    def init_orientation_axes(self):
        self.orientation_axes = OrientationAxes(size=2.0)
        self.orientation_axes.setParent(self)
        self.orientation_axes.move(10, self.height() - 90)
        self.orientation_axes.show()
        self.orientation_axes.cameraChanged.connect(self.sync_main_view_to_orientation)

    def sync_main_view_to_orientation(self):
        if self.current_pos is None:
            center = QVector3D(0, 0, 0)
        else:
            center = QVector3D(*self.current_pos)

        opts = self.orientation_axes.opts
        dist = opts.get('distance', 2.0)
        azim = opts.get('azimuth', 60)
        elev = opts.get('elevation', 30)

        self.view.setCameraPosition(distance=dist, azimuth=azim, elevation=elev)
        self.view.opts['center'] = center
        self.view.update()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        x = 10
        if hasattr(self, 'orientation_axes'):
            y = self.height() - self.orientation_axes.height() - 10
            self.orientation_axes.move(x, y)

    def init_visualization(self):
        colors = np.array([
            [0, 1, 0, 1],  # Left - green
            [1, 1, 0, 1],  # Center - yellow
            [0, 1, 0, 1],  # Right - green
        ], dtype=float)
        self.scatter_green = gl.GLScatterPlotItem(
            pos=self.points, size=10, color=colors, pxMode=True)
        self.view.addItem(self.scatter_green)

        self.scatter_red = gl.GLScatterPlotItem(
            pos=np.array([self.current_pos]), size=10, color=(1, 0, 0, 1), pxMode=True)
        self.view.addItem(self.scatter_red)

        self.scatter_blue = gl.GLScatterPlotItem(
            pos=np.array([self.target_pos]), size=10, color=(0, 0, 1, 1), pxMode=True)
        self.view.addItem(self.scatter_blue)

        meshdata, normal = compute_plane_mesh(self.points)
        self.plane_mesh = GLMeshItem(
            meshdata=meshdata,
            smooth=False,
            color=(0.5, 0.5, 0.5, 0.25),
            drawEdges=True,
            edgeColor=(0, 0, 0, 0.8),
            glOptions='additive'
        )
        self.view.addItem(self.plane_mesh)

    def init_left_panel(self):
        self.view = SyncableGLViewWidget()
        self.view.setCameraPosition(distance=10, azimuth=270)
        self.main_layout.addWidget(self.view, stretch=2)
        self.view.cameraMoved.connect(self.sync_orientation_axes_to_main_view)

    def sync_orientation_axes_to_main_view(self, azimuth, elevation):
        if hasattr(self, 'orientation_axes'):
            distance = self.orientation_axes.opts.get('distance', 2.0)
            self.orientation_axes.setCameraPosition(distance=distance, azimuth=azimuth, elevation=elevation)
            self.orientation_axes.update()


    def init_right_panel(self):
        self.right_layout = QVBoxLayout()
        self.main_layout.addLayout(self.right_layout, stretch=1)

        self.init_global_points_section()
        self.init_heart_frame_section()
        self.init_jog_controls()
        self.init_target_input()
        self.init_buttons_and_status()

    def init_global_points_section(self):
        group = QGroupBox("Global Points")
        layout = QVBoxLayout()
        self.global_points_coord_grid = QGridLayout()
        grid = self.global_points_coord_grid
        group.setLayout(layout)
        layout.addLayout(grid)

        labels = ["Left", "Center", "Right"]
        self.point_labels_global = []

        for i, label_text in enumerate(labels):
            lbl_name = QLabel(label_text)
            lbl_name.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            lbl_colon = QLabel(":")
            lbl_colon.setAlignment(Qt.AlignCenter)
            lbl_colon.setFixedWidth(10)
            colon_font = QFont()
            colon_font.setBold(True)
            lbl_colon.setFont(colon_font)
            lbl_coord = QLabel("(0.000, 0.000, 0.000)")
            lbl_coord.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

            grid.addWidget(lbl_name, i, 0)
            grid.addWidget(lbl_colon, i, 1)
            grid.addWidget(lbl_coord, i, 2)

            self.point_labels_global.append(lbl_coord)

        self.right_layout.addWidget(group)

    def init_heart_frame_section(self):
        group = QGroupBox("Heart Frame of Reference")
        layout = QVBoxLayout()
        grid = QGridLayout()
        group.setLayout(layout)
        layout.addLayout(grid)

        labels = ["Left", "Center", "Right"]
        self.point_labels_heart = []

        for i, label_text in enumerate(labels):
            lbl_name = QLabel(label_text)
            lbl_name.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            lbl_colon = QLabel(":")
            lbl_colon.setAlignment(Qt.AlignCenter)
            lbl_colon.setFixedWidth(10)
            colon_font = QFont()
            colon_font.setBold(True)
            lbl_colon.setFont(colon_font)
            lbl_coord = QLabel("(0.000, 0.000, 0.000)")
            lbl_coord.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

            grid.addWidget(lbl_name, i, 0)
            grid.addWidget(lbl_colon, i, 1)
            grid.addWidget(lbl_coord, i, 2)

            self.point_labels_heart.append(lbl_coord)

        target_label = QLabel("Target")
        target_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        target_colon = QLabel(":")
        target_colon.setAlignment(Qt.AlignCenter)
        target_colon.setFixedWidth(10)
        target_colon.setFont(colon_font)
        self.target_heart_label = QLabel("(0.000, 0.000, 0.000)")
        self.target_heart_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

        grid.addWidget(target_label, 3, 0)
        grid.addWidget(target_colon, 3, 1)
        grid.addWidget(self.target_heart_label, 3, 2)

        self.right_layout.addWidget(group)

    def init_action_buttons(self):
        action_group = QGroupBox("System Controls")
        action_layout = QVBoxLayout()

        self.end_button = QPushButton("End")
        self.end_button.clicked.connect(self.end_system)

        self.recenter_button = QPushButton("Fit to Screen")
        self.recenter_button.clicked.connect(self.recenter_view)

        action_layout.addWidget(self.end_button)
        action_layout.addWidget(self.recenter_button)

        action_group.setLayout(action_layout)
        self.right_layout.addWidget(action_group)

    def init_jog_controls(self):
        group = QGroupBox("Jog Controls (Joystick & WASD & Z +/-)")
        layout = QGridLayout()
        group.setLayout(layout)

        btn_up = QPushButton("w")
        btn_up.clicked.connect(lambda: self.jog_axis(1, 1))
        btn_left = QPushButton("a")
        btn_left.clicked.connect(lambda: self.jog_axis(0, -1))
        btn_right = QPushButton("s")
        btn_right.clicked.connect(lambda: self.jog_axis(2, 1))
        btn_down = QPushButton("d")
        btn_down.clicked.connect(lambda: self.jog_axis(1, -1))
        home_button = QPushButton("Home")
        home_button.clicked.connect(self.home_to_centroid)
        btn_z_plus = QPushButton("+")
        btn_z_plus.clicked.connect(lambda: self.jog_axis(2, 1))
        btn_z_minus = QPushButton("−")
        btn_z_minus.clicked.connect(lambda: self.jog_axis(2, -1))

        layout.addWidget(btn_up, 0, 1)
        layout.addWidget(btn_left, 1, 0)
        layout.addWidget(btn_right, 1, 2)
        layout.addWidget(btn_down, 1, 1)
        layout.addWidget(home_button, 2, 1)
        layout.addWidget(btn_z_plus, 2, 0)
        layout.addWidget(btn_z_minus, 2, 2)

        self.right_layout.addWidget(group)

    def init_target_input(self):
        group = QGroupBox("Target Position Input")
        layout = QGridLayout()
        group.setLayout(layout)

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
            layout.addWidget(axis_label, i, 0)
            layout.addWidget(curr_label, i, 1)
            layout.addWidget(arrow_label, i, 2)
            layout.addWidget(edit, i, 3)

            self.current_pos_labels[axis] = curr_label
            self.inputs[axis] = edit

        self.right_layout.addWidget(group)

    def init_buttons_and_status(self):
        buttons_layout = QHBoxLayout()
        self.move_button = QPushButton("Move to Position")
        self.move_button.clicked.connect(self.move_to_position)
        buttons_layout.addWidget(self.move_button)
        self.right_layout.addLayout(buttons_layout)

        status_layout = QHBoxLayout()
        self.status_prefix_label = QLabel("Status:")
        self.status_prefix_label.setStyleSheet("color: black;")
        self.status_prefix_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.status_value_label = QLabel("Unknown")
        self.status_value_label.setStyleSheet("color: black;")
        self.status_value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        status_layout.addWidget(self.status_prefix_label)
        status_layout.addWidget(self.status_value_label)
        self.right_layout.addLayout(status_layout)

        self.retry_button = QPushButton("Retry Shared Memory")
        self.retry_button.clicked.connect(self.try_connect_shared_memory)
        self.retry_button.hide()
        self.right_layout.addWidget(self.retry_button)

        self.right_layout.addStretch()

    def try_connect_shared_memory(self, first=False):
        try:
            self.read_shm = shared_memory.SharedMemory(
                name="Local\\CPPToPy", size=STATUS_STRUCT_SIZE)
            self.retry_button.hide()
            if not self.init_read_timer:
                self.read_timer = QTimer(self)
                self.read_timer.timeout.connect(self.update_status_from_shared_memory)
                self.read_timer.start(500)
                self.init_read_timer = True
            if not first:
                self.status_value_label.setText("Shared memory attached!")
        except FileNotFoundError:
            self.read_shm = None
            self.retry_button.show()
            if not first:
                QMessageBox.warning(self, "Shared Memory",
                                    "Shared memory segment not found. Please ensure process is running and retry.")

    def redraw_if_needed(self):
        if self._data_changed:
            self.scatter_red.setData(pos=np.array([self.current_pos]))
            self.scatter_blue.setData(pos=np.array([self.target_pos]))
            self.view.update()
            self._data_changed = False

    def update_status_from_shared_memory(self):
        try:
            if self.read_shm:
                status_data = bytes(self.read_shm.buf[:STATUS_STRUCT_SIZE])
                unpacked = struct.unpack(STATUS_STRUCT_FORMAT, status_data)

                points = np.array([
                    [unpacked[0], unpacked[1], unpacked[2]],  # Left
                    [unpacked[3], unpacked[4], unpacked[5]],  # Center
                    [unpacked[6], unpacked[7], unpacked[8]],  # Right
                    [unpacked[9], unpacked[10], unpacked[11]]  # MP
                ])
                status_bytes = unpacked[12]
                term_pos = status_bytes.find(b'\x00')
                status_str = status_bytes if term_pos == -1 else status_bytes[:term_pos]
                status = status_str.decode('utf-8')

                self.points = points[:3]
                self.current_pos = points[3]

                centroid = np.mean(self.points, axis=0)
                y_axis = self.points[1] - centroid
                y_axis /= np.linalg.norm(y_axis)
                v = self.points[0] - centroid
                v_proj = v - np.dot(v, y_axis) * y_axis
                x_axis = v_proj / np.linalg.norm(v_proj)
                z_axis = np.cross(x_axis, y_axis)
                R = np.column_stack((x_axis, y_axis, z_axis))
                points_centered = self.points - centroid
                transformed_points = (R.T @ points_centered.T).T

                for i, p in enumerate(transformed_points):
                    self.point_labels_heart[i].setText(f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")

                target_centered = self.target_pos - centroid
                target_transformed = R.T @ target_centered
                self.target_heart_label.setText(
                    f"({target_transformed[0]:.3f}, {target_transformed[1]:.3f}, {target_transformed[2]:.3f})"
                )
                self.target_heart_label.repaint()

                self.scatter_green.setData(pos=self.points)
                self.scatter_red.setData(pos=np.array([self.current_pos]))
                self.view.removeItem(self.plane_mesh)
                meshdata, _ = compute_plane_mesh(self.points)
                self.plane_mesh = GLMeshItem(
                    meshdata=meshdata,
                    smooth=False,
                    color=(0.5, 0.5, 0.5, 0.25),
                    drawEdges=True,
                    edgeColor=(0, 0, 0, 0.8),
                    glOptions='additive'
                )
                self.view.addItem(self.plane_mesh)

                coord_grid_layout = self.global_points_coord_grid
                for i, p in enumerate(self.points):
                    lbl_coord = coord_grid_layout.itemAtPosition(i, 2).widget()
                    lbl_coord.setText(f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")

                self.view.update()

                if status == "exit":
                    self.read_shm.close()
                    self.read_shm = None
                    self.status_value_label.setText("Shared memory closed by C++ (exit)")
                    self.status_value_label.setStyleSheet("color: red;")
                    self.retry_button.show()
                    return
                elif len(status) == 0:
                    self.status_value_label.setText("N/A")
                    self.status_value_label.setStyleSheet("color: red;")
                else:
                    self.status_value_label.setText(status)
                    self.status_value_label.setStyleSheet("color: black;")
                self.retry_button.hide()
            else:
                self.retry_button.show()
                self.status_value_label.setText("Shared memory missing, please reconnect")
                self.status_value_label.setStyleSheet("color: red;")
        except Exception as e:
            print(e)
            self.retry_button.show()
            self.status_value_label.setText("Shared memory closed, please reconnect")
            self.status_value_label.setStyleSheet("color: red;")

    def write_to_cpp(self, end_flag=False):
        if not self.write_shm:
            return
        try:
            data = struct.pack(MOTOR_STRUCT_FORMAT,
                               self.target_pos[0], self.target_pos[1], self.target_pos[2], True, end_flag)
            self.write_shm.buf[:MOTOR_STRUCT_SIZE] = data
        except Exception as e:
            print(f"Write error: {e}")

    def update_inputs_from_target(self):
        for axis in ['X', 'Y', 'Z']:
            idx = {'X': 0, 'Y': 1, 'Z': 2}[axis]
            val = self.target_pos[idx]
            self.inputs[axis].blockSignals(True)
            self.inputs[axis].setText(f"{val:.3f}")
            self.inputs[axis].blockSignals(False)

    def jog_axis(self, axis, direction):
        if self.points is None or self.target_pos is None:
            return

        R, centroid = compute_heart_frame_transform(self.points)
        increment = 0.01
        jog_heart = np.zeros(3)
        jog_heart[axis] = direction * increment
        jog_global = R @ jog_heart
        self.target_pos += jog_global
        self._data_changed = True
        self.update_inputs_from_target()
        self.update_labels()

    def move_to_position(self):
        if self.target_pos is not None:
            if not validate_position(self.target_pos, self.points):
                QMessageBox.warning(
                    self,
                    "Invalid Position",
                    "Moving platform is outside the triangle formed by L, C, and R.\n"
                    "Please choose a position within the bounds."
                )
                return
            else:
                self.write_to_cpp()
                self.current_pos = self.target_pos.copy()
                self._data_changed = True
                self.update_labels()

    def init_shared_memory(self):
        try:
            self.write_shm = shared_memory.SharedMemory(name="Local\\PyToCPP", create=True, size=MOTOR_STRUCT_SIZE)
        except FileExistsError:
            self.write_shm = shared_memory.SharedMemory(name="Local\\PyToCPP")

        self.read_shm = None
        self.init_read_timer = False
        self.try_connect_shared_memory(first=True)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status_from_shared_memory)
        self.status_timer.start(100)

        self.redraw_timer = QTimer(self)
        self.redraw_timer.timeout.connect(self.redraw_if_needed)
        self.redraw_timer.start(33)

    def home_to_centroid(self):
        if self.points is not None:
            self.target_pos = np.mean(self.points, axis=0)
            self._data_changed = True
            self.update_inputs_from_target()
            self.update_labels()
            self.move_to_position()

    def recenter_view(self):
        if self.current_pos is None:
            return
        distance = self.orientation_axes.opts.get('distance', 2.0)

        # Fixed azimuth and elevation as defaults (can adjust as needed)
        default_azimuth = 270
        default_elevation = 30

        center = QVector3D(*self.current_pos)

        # Set main view camera to use orientation axes zoom (distance)
        self.view.setCameraPosition(distance=distance,
                                    azimuth=default_azimuth,
                                    elevation=default_elevation)
        self.view.opts['center'] = center
        self.view.update()

        # Also sync orientation axes camera rotation but keep its zoom unchanged
        if hasattr(self, 'orientation_axes'):
            self.orientation_axes.setCameraPosition(distance=distance,
                                                    azimuth=default_azimuth,
                                                    elevation=default_elevation)

    def update_labels(self):
        if self.current_pos is not None:
            self.current_pos_labels['X'].setText(f"{self.current_pos[0]:.3f}")
            self.current_pos_labels['Y'].setText(f"{self.current_pos[1]:.3f}")
            self.current_pos_labels['Z'].setText(f"{self.current_pos[2]:.3f}")
        else:
            self.current_pos_labels['X'].setText("N/A")
            self.current_pos_labels['Y'].setText("N/A")
            self.current_pos_labels['Z'].setText("N/A")

    def end_system(self):
        self.write_to_cpp(end_flag=True)
        self.close()

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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
