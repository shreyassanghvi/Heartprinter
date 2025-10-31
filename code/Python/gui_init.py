import sys
import os
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QLabel,
    QGroupBox, QPushButton, QLineEdit, QMessageBox,
)
from PyQt5.QtGui import QPalette, QColor, QFont, QVector3D
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData, GLMeshItem
from multiprocessing import shared_memory

import struct
import time
from dataclasses import dataclass

@dataclass
class MotorCommand:
    target_x: float
    target_y: float
    target_z: float
    execute: bool
    exit: bool

MOTOR_STRUCT_FORMAT = 'ddd??5x'
MOTOR_STRUCT_SIZE = struct.calcsize(MOTOR_STRUCT_FORMAT)

@dataclass
class HeartprinterStatus:
    current_baseL_x: float
    current_baseL_y: float
    current_baseL_z: float
    current_baseC_x: float
    current_baseC_y: float
    current_baseC_z: float
    current_baseR_x: float
    current_baseR_y: float
    current_baseR_z: float
    current_baseMP_x: float
    current_baseMP_y: float
    current_baseMP_z: float
    status: str

STATUS_STRUCT_FORMAT = 'dddddddddddd6s2x'
STATUS_STRUCT_SIZE = struct.calcsize(STATUS_STRUCT_FORMAT)

def read_all_points(filename):
    """
    Read 3D coordinate points from a text file.
    
    Expects a file with lines containing X, Y, Z coordinates separated by spaces or commas.
    Reads up to 4 points maximum for this application.
    
    Args:
        filename (str): Path to the input file containing coordinate data
        
    Returns:
        numpy.ndarray or None: Array of shape (4, 3) containing [x, y, z] coordinates
                              for each point
    """
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
    """
    Generate a 3D triangular mesh representation of a plane defined by three points.
    
    Creates a subdivided triangle that shows the plane surface with good lighting
    and visual depth cues, while staying within the triangle boundary.
    
    Args:
        points (numpy.ndarray): Array of shape (3, 3) containing 3 points that define the plane
                               Each point is [x, y, z] coordinates
    
    Returns:
        tuple: (MeshData, normal_vector)
            - MeshData: PyQtGraph mesh object ready for 3D rendering
            - normal_vector: Unit vector perpendicular to the plane surface
    """
    p0, p1, p2 = points[0], points[1], points[2]
    
    # Create two vectors along the edges of the triangle
    v1 = p1 - p0
    v2 = p2 - p0

    # Find vector normal to the plane 
    normal = np.cross(v1, v2)
    normal /= np.linalg.norm(normal)

    # Create subdivided triangular mesh using rectangular coordinates
    n_grid = 15  # Subdivision level
    
    vertices = []
    faces = []
    
    # Generate vertices using rectangular grid within triangle
    for i in range(n_grid + 1):
        for j in range(n_grid + 1 - i):  # This creates the triangular shape
            # Use normalized coordinates (0 to 1)
            u = i / n_grid  # Parameter along v1 direction
            v = j / n_grid  # Parameter along v2 direction
            
            # Calculate 3D position: start at p0, move along v1 and v2
            point = p0 + u * v1 + v * v2
            vertices.append(point)
    
    # Convert to numpy array
    vertices = np.array(vertices)
    
    # Generate faces connecting the vertices
    vertex_idx = 0
    for i in range(n_grid + 1):
        for j in range(n_grid + 1 - i):
            if i < n_grid and j < n_grid - i:
                # Calculate indices for current row and next row
                curr_row_start = vertex_idx
                next_row_start = curr_row_start + (n_grid + 1 - i)
                
                # First triangle of the quad
                faces.append([
                    curr_row_start,
                    curr_row_start + 1,
                    next_row_start
                ])
                
                # Second triangle (if not at the edge)
                if j < n_grid - i - 1:
                    faces.append([
                        curr_row_start + 1,
                        next_row_start + 1,
                        next_row_start
                    ])
            vertex_idx += 1
    
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
    """
    Main application window for 3D plane and point visualization.
    
    This class creates the primary user interface with:
    - Left panel: 3D OpenGL visualization of points and plane
    - Right panel: Control interface with coordinate display, jog controls, and input fields
    
    The application manages three types of points:
    - Green points (3): Fixed reference points that define a plane
    - Red point (1): Current actual position (moveable)  
    - Blue point (1): Target position (where user wants to move)
    
    User can control the target position via:
    - Jog buttons (incremental movement)
    - Keyboard shortcuts (WASD + Z/+/-)
    - Direct coordinate input
    - Home button (move to centroid of reference points)
    """
    def __init__(self):
        """
        Initialize the main window and set up the complete user interface.
        
        Creates the layout, loads point data from file, initializes the 3D visualization,
        and sets up all control widgets. Also handles initial positioning and display updates.
        
        Raises:
            SystemExit: If input file doesn't contain 4 valid points
        """
        super().__init__()
        self.setWindowTitle("Plane & Points Visualization with Joystick & Input")
        self.resize(1280, 720)

        # === LEFT PANEL: 3D VISUALIZATION ===
        main_layout = QHBoxLayout(self)
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=10, azimuth=270)
        main_layout.addWidget(self.view, stretch=2)

        # === RIGHT PANEL: CONTROL INTERFACE ===
        right_panel = QVBoxLayout()

        right_group = QGroupBox("Points Coordinates")
        self.right_layout = QVBoxLayout()
        coord_grid_layout = QGridLayout()
        self.point_labels_3green = []
        right_group.setLayout(self.right_layout)
        right_panel.addWidget(right_group)

        jog_group = QGroupBox("Jog Controls (Joystick & WASD & Z +/-)")
        jog_layout = QGridLayout()
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
        jog_layout.addWidget(btn_up, 0, 1)
        jog_layout.addWidget(btn_left, 1, 0)
        jog_layout.addWidget(btn_right, 1, 2)
        jog_layout.addWidget(btn_down, 1, 1)
        jog_layout.addWidget(home_button, 2, 1)
        jog_layout.addWidget(btn_z_plus, 2, 0)
        jog_layout.addWidget(btn_z_minus, 2, 2)
        jog_group.setLayout(jog_layout)
        right_panel.addWidget(jog_group)

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

        status_layout = QHBoxLayout()
        self.status_prefix_label = QLabel("Status:")
        self.status_prefix_label.setStyleSheet("color: black;")
        self.status_prefix_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.status_value_label = QLabel("Unknown")
        self.status_value_label.setStyleSheet("color: black;")
        self.status_value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        status_layout.addWidget(self.status_prefix_label)
        status_layout.addWidget(self.status_value_label)
        right_panel.addLayout(status_layout)

        # -- Retry Button Logic --
        self.retry_button = QPushButton("Retry Shared Memory")
        self.retry_button.clicked.connect(self.try_connect_shared_memory)
        self.retry_button.hide()  # Start hidden unless there's an error
        right_panel.addWidget(self.retry_button)

        right_panel.addStretch()
        main_layout.addLayout(right_panel, stretch=0)

        # -- End Button Logic --
        self.end_button = QPushButton("End")
        self.end_button.clicked.connect(self.end_system)
        right_panel.addWidget(self.end_button)
        self.recenter_button = QPushButton("Recenter View")
        self.recenter_button.clicked.connect(self.recenter_view)
        right_panel.addWidget(self.recenter_button)

        # === INITIALIZE APPLICATION DATA ===
        self.setLayout(main_layout)

        # Load points
        self.points = np.zeros((3, 3))      # Three reference points at origin
        self.fourth_point = np.zeros(3)     # Fourth point at origin
        self.current_pos = self.fourth_point.copy()
        self.target_pos = np.mean(self.points, axis=0)


        labels = ["Left", "Center", "Right"]
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

        # === CREATE 3D VISUALIZATION OBJECTS ===
        # Green scatter plot for 3 reference points
        self.scatter_green = gl.GLScatterPlotItem(
            pos=self.points, size=10, color=(0, 1, 0, 1), pxMode=True)
        self.view.addItem(self.scatter_green)
        # Red scatter plot for current position (moveable point)
        self.scatter_red = gl.GLScatterPlotItem(
            pos=np.array([self.current_pos]), size=15, color=(1, 0, 0, 1), pxMode=True)
        self.view.addItem(self.scatter_red)
        # Blue scatter plot for target position  
        self.scatter_blue = gl.GLScatterPlotItem(
            pos=np.array([self.target_pos]), size=15, color=(0, 0, 1, 1), pxMode=True)
        self.view.addItem(self.scatter_blue)
        # Semi-transparent plane mesh through the 3 reference points
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
        # Initialize UI with current values
        self.update_inputs_from_target()
        self.update_labels()
        self._data_changed = True

        # Try shared memory -- if not found, show retry button
        self.read_shm = None
        self.init_read_timer = False
        self.try_connect_shared_memory(first=True)

        # Create Python -> C++ shared memory for writing
        try:
            self.write_shm = shared_memory.SharedMemory(name="Local\\PyToCPP", create=True, size=MOTOR_STRUCT_SIZE)
            # self.write_shm.buf[:] = b'\0' * 1024
        except FileExistsError:
            self.write_shm = shared_memory.SharedMemory(name="Local\\PyToCPP")
        # Timer to read position/status updates from C++ via shared memory
        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status_from_shared_memory)
        self.status_timer.start(100)  # status update every 500 ms
        # Timer to update the 3D visualization 
        self.redraw_timer = QTimer(self)
        self.redraw_timer.timeout.connect(self.redraw_if_needed)
        self.redraw_timer.start(33)  # ~30 FPS


    def recenter_view(self):
        if self.points is not None:
            center_pos = np.mean(self.points, axis=0)
        else:
            center_pos = np.array([0, 0, 0])

        # Set the center of view (the point camera looks at)
        self.view.opts['center'] = QVector3D(center_pos[0], center_pos[1], center_pos[2])


        # Adjust camera distance, azimuth, and elevation as desired (no 'center' kwarg!)
        self.view.setCameraPosition(
            distance=50,    # adjust as needed
            azimuth=270,    # rotation around vertical axis
            elevation=20    # vertical tilt angle
        )


    def try_connect_shared_memory(self, first=False):
        try:
            self.read_shm = shared_memory.SharedMemory(name="Local\\CPPToPy", size=STATUS_STRUCT_SIZE)
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
                QMessageBox.warning(self, "Shared Memory", "Shared memory segment not found. Please ensure C++ process is running and retry.")
        print(self.read_shm)

    def redraw_if_needed(self):
        if self._data_changed:
            self.scatter_red.setData(pos=np.array([self.current_pos]))
            self.scatter_blue.setData(pos=np.array([self.target_pos]))
            self.view.update()
            self._data_changed = False

    def write_to_cpp(self, end_flag=False):
        if not self.write_shm:
            return
        try:
            data = struct.pack(MOTOR_STRUCT_FORMAT, self.target_pos[0], self.target_pos[1], self.target_pos[2], True, end_flag)
            self.write_shm.buf[:MOTOR_STRUCT_SIZE] = data
        except Exception as e:
            print(f"Write error: {e}")

    def update_status_from_shared_memory(self):
        try:
            if self.read_shm:
                status_data = bytes(self.read_shm.buf[:STATUS_STRUCT_SIZE])
                unpacked = struct.unpack(STATUS_STRUCT_FORMAT, status_data)
                points = np.array([
                    [unpacked[0], unpacked[1], unpacked[2]],   # Left
                    [unpacked[3], unpacked[4], unpacked[5]],   # Center
                    [unpacked[6], unpacked[7], unpacked[8]],   # Right
                    [unpacked[9], unpacked[10], unpacked[11]]  # MP (moving point)
                ])
                status_bytes = unpacked[12]
                term_pos = status_bytes.find(b'\x00')
                status_str = status_bytes if term_pos == -1 else status_bytes[:term_pos]
                status = status_str.decode('utf-8')


                print(f"Left X, Y, and Z: ({points[0]})")
                print(f"Center X, Y, and Z: ({points[1]})")
                print(f"Right X, Y, and Z: ({points[2]})")
                print(f"MP X, Y, and Z: ({points[3]})")
                self.points = points[:3]
                self.current_pos = points[3]
                self.scatter_green.setData(pos=self.points)  # Green points for Left, Center, Right
                self.scatter_red.setData(pos=np.array([self.current_pos]))
                coord_grid_layout = self.right_layout.itemAt(0).widget().layout()
                labels = ["Left", "Center", "Right"]
                for i, p in enumerate(self.points):
                    lbl_coord = coord_grid_layout.itemAtPosition(i, 2).widget()
                    lbl_coord.setText(f"({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})")
                self.view.update()
                print(f"Status: {status}")
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
        """
        Set target position to the centroid (geometric center) of the 3 reference points.
        
        This provides a "home" or neutral position that's equidistant from all 3 reference
        points and lies in the center of the defined plane. Updates the visualization,
        input fields, and labels, then immediately moves to that position.
        
        Side effects:
            - Updates target_pos to centroid coordinates
            - Moves blue target point in 3D view
            - Updates input field values
            - Refreshes all display labels  
            - Executes move to new target position
        """
        if self.points is not None:
            self.target_pos = np.mean(self.points, axis=0)
            self._data_changed = True
            self.update_inputs_from_target()
            self.update_labels()
            self.move_to_position()

    def validate_position(self, point):
        """
        Check if a point is inside the triangle formed by the three reference points.
        
        Uses the same algorithm as the C++ validator:
        1. Project the point onto the plane containing the triangle
        2. Calculate areas of sub-triangles formed by the projected point
        3. Check if sum of sub-triangle areas equals the original triangle area
        
        Args:
            point (numpy.ndarray): 3D point to validate [x, y, z]
            
        Returns:
            bool: True if point is inside triangle, False otherwise
        """
        if self.points is None or len(self.points) < 3:
            return False
            
        a, b, c = self.points[0], self.points[1], self.points[2]
        
        # Step 1: Project point onto the plane containing triangle ABC
        
        # Find the plane normal vector using cross product of two triangle edges
        ab = b - a  # Vector AB
        ac = c - a  # Vector AC
        
        # Normal vector = AB × AC
        normal = np.cross(ab, ac)
        
        # Normalize the normal vector
        normal_length = np.linalg.norm(normal)
        if normal_length < 0.000001:
            return False  # Degenerate triangle (collinear points)
        normal = normal / normal_length
        
        # Find the projection of point onto the plane
        # Distance from point to plane = (point-A) · normal
        ap = point - a
        distance_to_plane = np.dot(ap, normal)
        
        # Projected point = point - distance * normal
        proj_point = point - distance_to_plane * normal
        
        # Step 2: Calculate areas using the projected point
        
        # Area of original triangle ABC
        area_abc = self.triangle_area(a, b, c)
        
        # Areas of three sub-triangles formed by projected point
        area_pab = self.triangle_area(proj_point, a, b)
        area_pbc = self.triangle_area(proj_point, b, c)
        area_pca = self.triangle_area(proj_point, c, a)
        
        # Step 3: Check if sum of sub-triangle areas equals the original triangle area
        sum_of_sub_areas = area_pab + area_pbc + area_pca
        area_difference = abs(sum_of_sub_areas - area_abc)
        
        # Point is inside triangle if the areas match within tolerance
        return area_difference <= 0.000001

    def triangle_area(self, p1, p2, p3):
        """
        Calculate the area of a triangle using cross product.
        
        Args:
            p1, p2, p3 (numpy.ndarray): Three points defining the triangle
            
        Returns:
            float: Area of the triangle
        """
        # Vector from p1 to p2 and p1 to p3
        v1 = p2 - p1
        v2 = p3 - p1
        
        # Cross product gives area vector, magnitude is 2 * area
        cross_product = np.cross(v1, v2)
        return 0.5 * np.linalg.norm(cross_product)

    def move_to_position(self):
        """
        Execute movement: set current position to match target position.
            - Moves red point in 3D visualization to target location
            - Updates position display labels
        """
        if self.target_pos is not None:
            # Validate position before moving
            if not self.validate_position(self.target_pos):
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

    def end_system(self):
        self.write_to_cpp(end_flag=True)
        self.close()

    def line_edit_changed(self, axis, widget):
        """
        Handle changes to coordinate input fields.
        
        Called when user finishes editing a coordinate input field (X, Y, or Z).
        Validates the input, updates the target position, and refreshes the visualization.
        Shows error dialog for invalid input and reverts to previous value.
        
        Args:
            axis (str): Which axis was changed ('X', 'Y', or 'Z')
            widget (QLineEdit): The input field widget that was modified
            
        Side effects:
            - Updates target_pos coordinate for specified axis
            - Moves blue target point in 3D view
            - Updates display labels
            - Shows error dialog for invalid input
            - Reverts input field on error
        """
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
        """
        Update coordinate input fields to match current target position.
        
        Fills the X, Y, Z input fields with the current target position coordinates.
        Temporarily blocks signals to prevent triggering change handlers during update.
        
        Side effects:
            - Updates text in all three coordinate input fields
            - Formats coordinates to 3 decimal places
        """
        for axis in ['X', 'Y', 'Z']:
            idx = {'X': 0, 'Y': 1, 'Z': 2}[axis]
            val = self.target_pos[idx]
            self.inputs[axis].blockSignals(True)
            self.inputs[axis].setText(f"{val:.3f}")
            self.inputs[axis].blockSignals(False)

    def jog_axis(self, axis, direction):
        """
        Perform incremental movement (jogging) along a specified axis.
        
        Moves the target position by a fixed increment (0.1 units) in the specified
        direction along the specified axis. This provides fine control for positioning.
        
        Args:
            axis (int): Axis to move along (0=X, 1=Y, 2=Z)
            direction (int): Direction to move (-1=negative, +1=positive)
            
        Side effects:
            - Updates target_pos coordinate by 0.1 units in specified direction
            - Moves blue target point in 3D view
            - Updates input fields with new target coordinates
            - Updates display labels
        """
        if self.target_pos is not None:
            self.target_pos[axis] += direction * 0.1
            self._data_changed = True
            self.update_inputs_from_target()
            self.update_labels()

    def keyPressEvent(self, event):
        """
        Handle keyboard input for joystick-style control.
        
        Provides keyboard shortcuts for common operations:
        - W/S: Move along Y axis (forward/backward)
        - A/D: Move along X axis (left/right)  
        - +/-: Move along Z axis (up/down)
        - Enter: Execute move to target position
        - Z: Home to centroid position
        
        Args:
            event (QKeyEvent): Keyboard event containing key information
            
        Side effects:
            - Calls appropriate movement or action methods based on key pressed
            - Falls back to parent class handling for unrecognized keys
        """
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
        """
        Update all position display labels with current coordinate values.
        
        Refreshes the red current position labels that appear next to each input field.
        Shows current X, Y, Z coordinates formatted to 3 decimal places, or "N/A"
        if position data is not available.
        
        Side effects:
            - Updates text in current position labels for X, Y, Z axes
            - Formats coordinates to 3 decimal places
            - Shows "N/A" if current_pos is None
        """
        if self.current_pos is not None:
            self.current_pos_labels['X'].setText(f"{self.current_pos[0]:.3f}")
            self.current_pos_labels['Y'].setText(f"{self.current_pos[1]:.3f}")
            self.current_pos_labels['Z'].setText(f"{self.current_pos[2]:.3f}")
        else:
            self.current_pos_labels['X'].setText("N/A")
            self.current_pos_labels['Y'].setText("N/A")
            self.current_pos_labels['Z'].setText("N/A")

if __name__ == "__main__":
    """
    Application entry point.
    
    Creates the PyQt5 application, instantiates the main window, shows it,
    and starts the event loop. The application will run until the user closes
    the window or an error forces exit.
    """
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec_())
