import sys
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QSpinBox, QPushButton, QLabel, QGroupBox
)
import pyqtgraph.opengl as gl
import numpy as np

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Joystick Jog Controls with 3D Point Cloud Map")
        self.resize(1200, 600)

        main_layout = QHBoxLayout(self)

        # 3D Point Cloud (white background)
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=40, azimuth=270)

        grid = gl.GLGridItem()
        grid.setColor((200,200,200,100))
        self.view.addItem(grid)

        # Origin (red dot)
        origin_pos = np.array([[0, 0, 0]])
        origin_marker = gl.GLScatterPlotItem(
            pos=origin_pos, size=14, color=(1, 0, 0, 1), pxMode=True)
        self.view.addItem(origin_marker)

        # Movable marker (blue)
        self.current_pos = np.array([0, 0, 0])
        self.marker = gl.GLScatterPlotItem(
            pos=np.array([self.current_pos]), size=14, color=(0, 0, 1, 1), pxMode=True)
        self.view.addItem(self.marker)

        # Point cloud (green dots)
        self.data_points = np.array([
            [10, 10, 0], [15, 5, 7], [7, -8, 9], [-10, -5, -12], [0, 0, 15]
        ])
        self.point_cloud = gl.GLScatterPlotItem(
            pos=self.data_points, size=14, color=(0, 1, 0, 1), pxMode=True)
        self.view.addItem(self.point_cloud)

        main_layout.addWidget(self.view, stretch=4)

        # Controls Layout
        controls_layout = QVBoxLayout()

        # Move to Position controls
        input_group = QGroupBox("Move to Position")
        input_layout = QHBoxLayout()
        self.x_input = QSpinBox(); self.x_input.setRange(-100, 100)
        self.y_input = QSpinBox(); self.y_input.setRange(-100, 100)
        self.z_input = QSpinBox(); self.z_input.setRange(-100, 100)
        input_layout.addWidget(QLabel("X")); input_layout.addWidget(self.x_input)
        input_layout.addWidget(QLabel("Y")); input_layout.addWidget(self.y_input)
        input_layout.addWidget(QLabel("Z")); input_layout.addWidget(self.z_input)
        move_btn = QPushButton("Move To")
        move_btn.clicked.connect(self.move_marker)
        input_layout.addWidget(move_btn)
        input_group.setLayout(input_layout)
        controls_layout.addWidget(input_group)

        # Joystick Jog Controls (grid)
        jog_group = QGroupBox("Jog Controls (Joystick & WASD)")
        grid_layout = QGridLayout()

        btn_up = QPushButton("↑"); btn_up.clicked.connect(lambda: self.jog_axis(1, 1))   # Invert Y
        btn_left = QPushButton("←"); btn_left.clicked.connect(lambda: self.jog_axis(0, -1))
        btn_center = QLabel("")  # Empty center label or for future icon
        btn_right = QPushButton("→"); btn_right.clicked.connect(lambda: self.jog_axis(0, 1))
        btn_down = QPushButton("↓"); btn_down.clicked.connect(lambda: self.jog_axis(1, -1)) # Invert Y

        # Place in grid
        grid_layout.addWidget(btn_up, 0, 1)
        grid_layout.addWidget(btn_left, 1, 0)
        grid_layout.addWidget(btn_center, 1, 1)
        grid_layout.addWidget(btn_right, 1, 2)
        grid_layout.addWidget(btn_down, 2, 1)
        jog_group.setLayout(grid_layout)
        controls_layout.addWidget(jog_group)

        main_layout.addLayout(controls_layout, stretch=1)
        self.setLayout(main_layout)
        self.setFocusPolicy(Qt.StrongFocus)

    def move_marker(self):
        self.current_pos = np.array([
            self.x_input.value(),
            self.y_input.value(),
            self.z_input.value()
        ])
        self.marker.setData(pos=np.array([self.current_pos]))

    def jog_axis(self, axis, direction):
        self.current_pos[axis] += direction
        self.marker.setData(pos=np.array([self.current_pos]))
        self.x_input.setValue(int(self.current_pos[0]))
        self.y_input.setValue(int(self.current_pos[1]))
        self.z_input.setValue(int(self.current_pos[2]))

    def keyPressEvent(self, event):
        key = event.key()
        # WASD keys, invert Y
        if key == Qt.Key_W:      # Up (Y-1)
            self.jog_axis(1, 1)
        elif key == Qt.Key_S:    # Down (Y+1)
            self.jog_axis(1, -1)
        elif key == Qt.Key_A:    # Left (X-1)
            self.jog_axis(0, -1)
        elif key == Qt.Key_D:    # Right (X+1)
            self.jog_axis(0, 1)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
