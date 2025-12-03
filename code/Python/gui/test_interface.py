import sys
import struct
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QGridLayout, QLabel,
    QLineEdit, QPushButton, QComboBox, QHBoxLayout, QGroupBox, QMessageBox
)
from PyQt5.QtCore import QTimer
from multiprocessing import shared_memory

STATUS_STRUCT_FORMAT = 'dddddddddddd6s2x'
STATUS_STRUCT_SIZE = struct.calcsize(STATUS_STRUCT_FORMAT)

MOTOR_STRUCT_FORMAT = 'ddd??5x'
MOTOR_STRUCT_SIZE = struct.calcsize(MOTOR_STRUCT_FORMAT)

class SharedMemoryWriter(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Shared Memory 3D Points & Status Input with Readback")
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        # Coordinate inputs grid
        self.coord_grid = QGridLayout()
        self.inputs = []  # List of QLineEdit for all point coordinates
        labels = ["L_X", "L_Y", "L_Z", "C_X", "C_Y", "C_Z",
                  "R_X", "R_Y", "R_Z", "MP_X", "MP_Y", "MP_Z"]
        for i, label_text in enumerate(labels):
            label = QLabel(label_text)
            self.coord_grid.addWidget(label, i, 0)
            line_edit = QLineEdit("0.0")
            self.coord_grid.addWidget(line_edit, i, 1)
            self.inputs.append(line_edit)
        self.layout.addLayout(self.coord_grid)

        # Status Dropdown
        self.status_label = QLabel("Status:")
        self.status_combo = QComboBox()
        self.status_combo.addItems(["ready", "init", "run"])
        self.layout.addWidget(self.status_label)
        self.layout.addWidget(self.status_combo)

        # Start/Stop Buttons
        self.start_button = QPushButton("Start Writing")
        self.stop_button = QPushButton("Stop Writing")
        self.stop_button.setEnabled(False)
        self.layout.addWidget(self.start_button)
        self.layout.addWidget(self.stop_button)
        self.start_button.clicked.connect(self.start_writing)
        self.stop_button.clicked.connect(self.stop_writing)

        # Group box for displaying MotorCommand readback
        readback_group = QGroupBox("Readback from PyToCPP (MotorCommand)")
        readback_layout = QGridLayout()
        readback_group.setLayout(readback_layout)
        self.layout.addWidget(readback_group)

        self.labels_readback = {}
        fields = ['Target X', 'Target Y', 'Target Z', 'Execute', 'Exit']
        for i, field in enumerate(fields):
            readback_layout.addWidget(QLabel(field + ":"), i, 0)
            lbl = QLabel("--")
            readback_layout.addWidget(lbl, i, 1)
            self.labels_readback[field] = lbl

        # Group box for motor position calculation
        motor_calc_group = QGroupBox("Motor Position Calculation")
        motor_calc_layout = QGridLayout()
        motor_calc_group.setLayout(motor_calc_layout)
        self.layout.addWidget(motor_calc_group)

        # Cable length displays
        self.labels_motor_calc = {}
        motor_fields = ['Cable 0 Length (mm)', 'Cable 1 Length (mm)', 'Cable 2 Length (mm)',
                       'Motor 0 Steps', 'Motor 1 Steps', 'Motor 2 Steps']
        for i, field in enumerate(motor_fields):
            motor_calc_layout.addWidget(QLabel(field + ":"), i, 0)
            lbl = QLabel("--")
            motor_calc_layout.addWidget(lbl, i, 1)
            self.labels_motor_calc[field] = lbl

        # Retry button for read shared memory
        self.retry_button = QPushButton("Retry PyToCPP Shared Memory")
        self.retry_button.hide()
        self.retry_button.clicked.connect(self.try_connect_shm_read)
        self.layout.addWidget(self.retry_button)

        # Timer for writing
        self.timer = QTimer()
        self.timer.timeout.connect(self.write_shared_memory)
        # Timer for reading
        self.read_timer = QTimer()
        self.read_timer.timeout.connect(self.read_shared_memory)

        # Open or create shared memory for writing
        try:
            self.shm_write = shared_memory.SharedMemory(name="Local\\CPPToPy", create=True, size=STATUS_STRUCT_SIZE)
        except FileExistsError:
            self.shm_write = shared_memory.SharedMemory(name="Local\\CPPToPy")

        # Attempt to open shared memory for reading motor command
        self.shm_read = None
        self.try_connect_shm_read()
        if self.shm_read:
            self.read_timer.start(100)  # read at 10 Hz

    def try_connect_shm_read(self):
        try:
            if self.shm_read:
                self.shm_read.close()
            self.shm_read = shared_memory.SharedMemory(name="Local\\PyToCPP", create=False, size=MOTOR_STRUCT_SIZE)
            self.retry_button.hide()
            if not self.read_timer.isActive():
                self.read_timer.start(100)
        except FileNotFoundError:
            self.shm_read = None
            self.retry_button.show()
            QMessageBox.warning(self, "Shared Memory Missing",
                                "Shared memory segment 'Local\\PyToCPP' not found. Please start the producer and retry.")

    def start_writing(self):
        self.timer.start(33)  # ~30Hz
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)

    def stop_writing(self):
        self.timer.stop()
        self.start_button.setEnabled(True)
        self.stop_button.setEnabled(False)

    def write_shared_memory(self):
        try:
            coords = []
            for edit in self.inputs:
                try:
                    val = float(edit.text())
                except ValueError:
                    val = 0.0
                coords.append(val)
            if len(coords) != 12:
                return

            points = coords[:12]

            status_text = self.status_combo.currentText().encode('utf-8')
            status_bytes = status_text + b'\x00' * (6 - len(status_text))
            status_bytes = status_bytes[:6]

            packed = struct.pack(STATUS_STRUCT_FORMAT,
                                 points[0], points[1], points[2],   # Left point
                                 points[3], points[4], points[5],   # Center point
                                 points[6], points[7], points[8],   # Right point
                                 points[9], points[10], points[11], # Moving point
                                 status_bytes)

            self.shm_write.buf[:STATUS_STRUCT_SIZE] = packed
        except Exception as e:
            print(f"Error writing shared memory: {e}")

    def calculate_motor_positions(self, target_x, target_y, target_z):
        try:
            # TODO: This is calculating the motor positions using the global position. This is wrong. It needs to calculate the motor positions using the
            #   Local coordinates of the moving point. Otherwise, we're going to be letting out way too much wire.
            # Get base positions from inputs
            static_base_positions = [
                np.array([float(self.inputs[0].text()), float(self.inputs[1].text()), float(self.inputs[2].text())]),   # Base 0 (Left)
                np.array([float(self.inputs[3].text()), float(self.inputs[4].text()), float(self.inputs[5].text())]),   # Base 1 (Center)
                np.array([float(self.inputs[6].text()), float(self.inputs[7].text()), float(self.inputs[8].text())])    # Base 2 (Right)
            ]

            # Target position (from MotorCommand in C++)
            target_pos = np.array([target_x, target_y, target_z])

            PULLEY_DIAMETER_MM = 30.0

            # mmToDynamixelUnits()
            steps_per_mm = 4096.0 / (np.pi * PULLEY_DIAMETER_MM)

            # Calculate for each motor
            cable_lengths = []
            motor_destinations = []

            for i in range(3):
                # Vector from base to target
                dx = target_pos[0] - static_base_positions[i][0]
                dy = target_pos[1] - static_base_positions[i][1]
                dz = target_pos[2] - static_base_positions[i][2]

                # Distance from target to base
                cable_length = np.sqrt(dx**2 + dy**2 + dz**2)
                cable_lengths.append(cable_length)

                # Convert to steps
                motor_dest = int(cable_length * steps_per_mm)
                motor_destinations.append(motor_dest)

                # Update display
                self.labels_motor_calc[f'Cable {i} Length (mm)'].setText(f"{cable_length:.3f}")
                self.labels_motor_calc[f'Motor {i} Steps'].setText(f"{motor_dest}")

            return cable_lengths, motor_destinations

        except ValueError as e:
            print(f"Error calculating motor positions: Invalid input values - {e}")
            return None, None
        except Exception as e:
            print(f"Error calculating motor positions: {e}")
            return None, None

    def read_shared_memory(self):
        if not self.shm_read:
            return
        try:
            data = bytes(self.shm_read.buf[:MOTOR_STRUCT_SIZE])
            unpacked = struct.unpack(MOTOR_STRUCT_FORMAT, data)
            self.labels_readback['Target X'].setText(f"{unpacked[0]:.3f}")
            self.labels_readback['Target Y'].setText(f"{unpacked[1]:.3f}")
            self.labels_readback['Target Z'].setText(f"{unpacked[2]:.3f}")
            self.labels_readback['Execute'].setText(str(unpacked[3]))
            self.labels_readback['Exit'].setText(str(unpacked[4]))

            # Calculate motor positions from target
            target_x, target_y, target_z = unpacked[0], unpacked[1], unpacked[2]
            self.calculate_motor_positions(target_x, target_y, target_z)

            # If execute is True, update MP input fields from shared memory values
            if unpacked[3]:  # execute == True
                # indexes 9,10,11 in self.inputs are MP_X, MP_Y, MP_Z according to your coord labels array
                self.inputs[9].setText(f"{unpacked[0]:.3f}")   # MP_X
                self.inputs[10].setText(f"{unpacked[1]:.3f}")  # MP_Y
                self.inputs[11].setText(f"{unpacked[2]:.3f}")  # MP_Z

        except Exception as e:
            print(f"Error reading shared memory: {e}")
            self.shm_read.close()
            self.shm_read = None
            self.retry_button.show()
            self.read_timer.stop()


    def closeEvent(self, event):
        self.timer.stop()
        self.read_timer.stop()
        if self.shm_write:
            self.shm_write.close()
            try:
                self.shm_write.unlink()
            except FileNotFoundError:
                pass
        if self.shm_read:
            self.shm_read.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SharedMemoryWriter()
    window.show()
    sys.exit(app.exec_())
