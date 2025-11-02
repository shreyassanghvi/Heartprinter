import sys
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QLabel,
    QGroupBox, QPushButton, QLineEdit, QMessageBox,
)
from PyQt5.QtGui import QPalette, QColor, QFont, QVector3D
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLMeshItem, MeshData
from multiprocessing import shared_memory
import struct

from compute import (
    MOTOR_STRUCT_FORMAT, MOTOR_STRUCT_SIZE,
    STATUS_STRUCT_FORMAT, STATUS_STRUCT_SIZE,
    compute_plane_mesh, validate_position, compute_heart_frame_transform
)


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

    def init_visualization(self):
        # Initialize 3D scatter plots for reference, current, and target points
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

        # Initialize the plane mesh for the 3 reference points
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
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=10, azimuth=270)
        self.main_layout.addWidget(self.view, stretch=2)

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

        # Target position in Heart Frame
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
        # Create an actions group box or layout
        action_group = QGroupBox("System Controls")
        action_layout = QVBoxLayout()

        self.end_button = QPushButton("End")
        self.end_button.clicked.connect(self.end_system)  # your existing clean-up method

        self.recenter_button = QPushButton("Fit to Screen")
        self.recenter_button.clicked.connect(self.recenter_view)

        action_layout.addWidget(self.end_button)
        action_layout.addWidget(self.recenter_button)

        action_group.setLayout(action_layout)

        # Add to right panel layout
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
                self.read_timer.start(500)  # Every 500 ms
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
        """
        Jog movement in Heart Frame coordinates, transformed to global coordinates.

        axis: int (0=X,1=Y,2=Z in Heart Frame)
        direction: int (+1 or -1)
        """
        if self.points is None or self.target_pos is None:
            return

        # Step 1: Compute Heart Frame transform
        R, centroid = compute_heart_frame_transform(self.points)  # R columns are Heart Frame axes

        # Step 2: Define jog vector in Heart Frame
        increment = 0.01  # jogging step size
        jog_heart = np.zeros(3)
        jog_heart[axis] = direction * increment  # e.g., +0.1 along Y axis of Heart Frame

        # Step 3: Convert jog vector to global coordinates
        jog_global = R @ jog_heart

        # Step 4: Apply jog in global frame to target_pos
        self.target_pos += jog_global

        # Mark data changed and update UI
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
        # Create Python -> C++ shared memory to write MotorCommand
        try:
            self.write_shm = shared_memory.SharedMemory(name="Local\\PyToCPP", create=True, size=MOTOR_STRUCT_SIZE)
        except FileExistsError:
            self.write_shm = shared_memory.SharedMemory(name="Local\\PyToCPP")

        # Shared memory to read status from C++ process
        self.read_shm = None
        self.init_read_timer = False
        self.try_connect_shared_memory(first=True)

        # Timer to read periodic updates from C++
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status_from_shared_memory)
        self.status_timer.start(100)  # 100ms updates

        # Timer to update 3D visualization
        self.redraw_timer = QTimer(self)
        self.redraw_timer.timeout.connect(self.redraw_if_needed)
        self.redraw_timer.start(33)  # ~30 FPS

    def home_to_centroid(self):
        if self.points is not None:
            self.target_pos = np.mean(self.points, axis=0)
            self._data_changed = True
            self.update_inputs_from_target()
            self.update_labels()
            self.move_to_position()

    def recenter_view(self):
        if self.points is None or len(self.points) < 3:
            return

        centroid = np.mean(self.points, axis=0)
        R, _ = compute_heart_frame_transform(self.points)
        plane_normal = R[:, 2]  # Z axis of Heart Frame (plane normal)
        heart_y_axis = R[:, 1]  # Y axis (center base vector)

        camera_distance = 5.0
        self.view.opts['center'] = QVector3D(*centroid)

        # Compute angle of Heart Frame Y axis in XY plane
        angle_y = np.degrees(np.arctan2(heart_y_axis[1], heart_y_axis[0]))

        # Compute angle of Heart Frame X axis in XY plane for roll correction
        heart_x_axis = R[:, 0]
        angle_x = np.degrees(np.arctan2(heart_x_axis[1], heart_x_axis[0]))

        # Base azimuth to point Y axis downward + 180 deg
        base_azimuth = (angle_y + 180) % 360

        # Approximate roll correction: difference between X axis and ideal frame
        # Adjust azimuth by the negative angle_x to 'roll' view flat
        adjusted_azimuth = (base_azimuth - angle_x) % 360

        # Set camera looking straight down
        self.view.setCameraPosition(distance=camera_distance,
                                    azimuth=adjusted_azimuth, elevation=90)



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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
