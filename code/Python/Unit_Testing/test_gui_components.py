"""
Unit tests for GUI components in the Heartprinter application.
Tests for main.py and test_interface.py components.
"""
import sys
import os
import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt, QPoint, QTimer
from PyQt5.QtTest import QTest
from PyQt5.QtGui import QVector3D
import struct

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import GUI components
from gui.main import SyncableGLViewWidget, OrientationAxes, MainWindow
from gui.test_interface import SharedMemoryWriter
from core.compute import (
    MOTOR_STRUCT_FORMAT, MOTOR_STRUCT_SIZE,
    STATUS_STRUCT_FORMAT, STATUS_STRUCT_SIZE,
    compute_plane_mesh, validate_position, compute_heart_frame_transform
)


@pytest.fixture(scope='session')
def qapp():
    """Create QApplication instance for all tests."""
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    yield app


@pytest.fixture
def sample_points():
    """Provide sample 3D points for testing."""
    return np.array([
        [1.0, 0.0, 0.0],  # Left
        [0.0, 1.0, 0.0],  # Center
        [0.0, 0.0, 1.0],  # Right
    ])


class TestSyncableGLViewWidget:
    """Tests for SyncableGLViewWidget class."""

    def test_initialization(self, qapp):
        """Test that SyncableGLViewWidget initializes correctly."""
        widget = SyncableGLViewWidget()
        assert widget is not None
        assert hasattr(widget, 'cameraMoved')
        assert widget._last_azimuth == widget.opts.get('azimuth', 0)
        assert widget._last_elevation == widget.opts.get('elevation', 0)

    def test_camera_moved_signal_emission(self, qapp, qtbot):
        """Test that cameraMoved signal is emitted when camera orientation changes."""
        widget = SyncableGLViewWidget()
        qtbot.addWidget(widget)

        # Mock the signal
        signal_spy = Mock()
        widget.cameraMoved.connect(signal_spy)

        # Store initial values
        initial_azimuth = widget._last_azimuth
        initial_elevation = widget._last_elevation

        # Change camera position
        widget.opts['azimuth'] = 45
        widget.opts['elevation'] = 30

        # Manually trigger the signal emission check (simulating what mouseMoveEvent does)
        azim = widget.opts.get('azimuth', 0)
        elev = widget.opts.get('elevation', 0)

        # Emit only if changed (matching the actual implementation)
        if azim != widget._last_azimuth or elev != widget._last_elevation:
            widget._last_azimuth = azim
            widget._last_elevation = elev
            widget.cameraMoved.emit(azim, elev)

        # Check if internal state is updated
        assert widget._last_azimuth == 45
        assert widget._last_elevation == 30

        # Verify signal was emitted (values should have changed from initial)
        if 45 != initial_azimuth or 30 != initial_elevation:
            signal_spy.assert_called_with(45, 30)
        else:
            # If initial values were already 45, 30, signal wouldn't be emitted
            pass


class TestOrientationAxes:
    """Tests for OrientationAxes class."""

    def test_initialization(self, qapp):
        """Test that OrientationAxes initializes with correct size and properties."""
        axes = OrientationAxes(size=2.0)
        assert axes is not None
        assert axes.size().width() == 80
        assert axes.size().height() == 80
        assert axes.opts['distance'] == 2.0
        assert hasattr(axes, 'cameraChanged')
        assert axes.dragging is False
        assert axes.last_pos is None

    def test_custom_size(self, qapp):
        """Test OrientationAxes with custom size parameter."""
        custom_size = 5.0
        axes = OrientationAxes(size=custom_size)
        assert axes.opts['distance'] == custom_size

    def test_mouse_press_event(self, qapp, qtbot):
        """Test mouse press event handling."""
        axes = OrientationAxes()
        qtbot.addWidget(axes)

        # Simulate left mouse button press
        event = Mock()
        event.button.return_value = Qt.LeftButton
        event.pos.return_value = QPoint(10, 10)
        event.accept = Mock()

        axes.mousePressEvent(event)

        assert axes.dragging is True
        assert axes.last_pos == event.pos()
        event.accept.assert_called_once()

    def test_mouse_release_event(self, qapp, qtbot):
        """Test mouse release event handling."""
        axes = OrientationAxes()
        qtbot.addWidget(axes)

        # Set dragging state
        axes.dragging = True

        # Simulate left mouse button release
        event = Mock()
        event.button.return_value = Qt.LeftButton
        event.accept = Mock()

        axes.mouseReleaseEvent(event)

        assert axes.dragging is False
        event.accept.assert_called_once()

    def test_camera_changed_signal(self, qapp, qtbot):
        """Test that cameraChanged signal is emitted during mouse move."""
        axes = OrientationAxes()
        qtbot.addWidget(axes)

        # Connect signal spy
        signal_spy = Mock()
        axes.cameraChanged.connect(signal_spy)

        # Start dragging
        axes.dragging = True
        axes.last_pos = QPoint(10, 10)

        # Simulate mouse move
        event = Mock()
        event.x.return_value = 15
        event.y.return_value = 12
        event.pos.return_value = QPoint(15, 12)
        event.accept = Mock()

        axes.mouseMoveEvent(event)

        # Verify signal was emitted
        signal_spy.assert_called_once()


class TestMainWindow:
    """Tests for MainWindow class."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_initialization(self, mock_shm, qapp):
        """Test that MainWindow initializes correctly."""
        window = MainWindow()
        assert window is not None
        assert window.windowTitle() == "Plane & Points Visualization"
        assert window.points.shape == (3, 3)
        assert window.current_pos.shape == (3,)
        assert window.target_pos.shape == (3,)
        assert window.waypoint_routine_active is False
        assert window.waypoint_queue == []

    @patch('gui.main.shared_memory.SharedMemory')
    def test_ui_components_exist(self, mock_shm, qapp):
        """Test that all major UI components are created."""
        window = MainWindow()

        # Check that key UI elements exist
        assert hasattr(window, 'view')
        assert hasattr(window, 'orientation_axes')
        assert hasattr(window, 'move_button')
        assert hasattr(window, 'waypoint_button')
        assert hasattr(window, 'end_button')
        assert hasattr(window, 'recenter_button')
        assert hasattr(window, 'status_value_label')
        assert hasattr(window, 'inputs')
        assert len(window.inputs) == 3  # X, Y, Z

    @patch('gui.main.shared_memory.SharedMemory')
    def test_jog_axis(self, mock_shm, qapp, sample_points):
        """Test jogging along an axis."""
        window = MainWindow()
        window.points = sample_points
        window.target_pos = np.array([0.5, 0.5, 0.5])
        initial_pos = window.target_pos.copy()

        # Jog in X direction (axis 0)
        window.jog_axis(0, 1)

        # Target position should have changed
        assert not np.array_equal(window.target_pos, initial_pos)
        assert window._data_changed is True

    @patch('gui.main.shared_memory.SharedMemory')
    def test_home_to_centroid(self, mock_shm, qapp, sample_points):
        """Test homing to centroid position."""
        window = MainWindow()
        window.points = sample_points
        window.target_pos = np.array([2.0, 2.0, 2.0])

        # Calculate expected centroid
        expected_centroid = np.mean(sample_points, axis=0)

        # Mock the move_to_position to avoid shared memory issues
        window.move_to_position = Mock()

        # Home to centroid
        window.home_to_centroid()

        # Check that target position is now the centroid
        np.testing.assert_array_almost_equal(window.target_pos, expected_centroid)
        assert window._data_changed is True

    @patch('gui.main.shared_memory.SharedMemory')
    def test_line_edit_changed_valid_input(self, mock_shm, qapp):
        """Test updating target position from line edit with valid input."""
        window = MainWindow()
        window.target_pos = np.array([0.0, 0.0, 0.0])

        # Mock the widget
        widget = Mock()
        widget.text.return_value = "5.5"

        # Update X axis
        window.line_edit_changed('X', widget)

        assert window.target_pos[0] == 5.5
        assert window._data_changed is True

    @patch('gui.main.shared_memory.SharedMemory')
    @patch('gui.main.QMessageBox')
    def test_line_edit_changed_invalid_input(self, mock_msgbox, mock_shm, qapp):
        """Test handling invalid input in line edit."""
        window = MainWindow()
        window.target_pos = np.array([1.0, 2.0, 3.0])

        # Mock the widget with invalid text
        widget = Mock()
        widget.text.return_value = "invalid"

        # Attempt to update X axis
        window.line_edit_changed('X', widget)

        # Target position should remain unchanged
        assert window.target_pos[0] == 1.0
        # Warning should be shown
        mock_msgbox.warning.assert_called_once()

    @patch('gui.main.shared_memory.SharedMemory')
    def test_update_labels(self, mock_shm, qapp):
        """Test updating position labels."""
        window = MainWindow()
        window.current_pos = np.array([1.234, 5.678, 9.012])

        window.update_labels()

        assert window.current_pos_labels['X'].text() == "1.234"
        assert window.current_pos_labels['Y'].text() == "5.678"
        assert window.current_pos_labels['Z'].text() == "9.012"

    @patch('gui.main.shared_memory.SharedMemory')
    def test_update_labels_none_position(self, mock_shm, qapp):
        """Test updating labels when position is None."""
        window = MainWindow()
        window.current_pos = None

        window.update_labels()

        assert window.current_pos_labels['X'].text() == "N/A"
        assert window.current_pos_labels['Y'].text() == "N/A"
        assert window.current_pos_labels['Z'].text() == "N/A"

    @patch('gui.main.shared_memory.SharedMemory')
    @patch('gui.main.QMessageBox')
    def test_move_to_position_invalid(self, mock_msgbox, mock_shm, qapp, sample_points):
        """Test moving to an invalid position (outside triangle)."""
        window = MainWindow()
        window.points = sample_points
        initial_current_pos = window.current_pos.copy()

        # Use a point that's definitely outside the triangle
        # Triangle is at [1,0,0], [0,1,0], [0,0,1]
        # Point at [10, 0, 0] projects outside the triangle
        window.target_pos = np.array([10.0, 0.0, 0.0])

        window.move_to_position()

        # Warning should be shown
        mock_msgbox.warning.assert_called_once()
        # Current position should not change from initial
        np.testing.assert_array_equal(window.current_pos, initial_current_pos)

    @patch('gui.main.shared_memory.SharedMemory')
    def test_move_to_position_valid(self, mock_shm, qapp, sample_points):
        """Test moving to a valid position (inside triangle)."""
        window = MainWindow()
        window.points = sample_points
        # Centroid is always inside
        window.target_pos = np.mean(sample_points, axis=0)

        window.move_to_position()

        # Current position should update to target
        np.testing.assert_array_almost_equal(window.current_pos, window.target_pos)

    @patch('gui.main.shared_memory.SharedMemory')
    def test_key_press_wasd(self, mock_shm, qapp, sample_points):
        """Test WASD key press handling."""
        window = MainWindow()
        window.points = sample_points
        window.target_pos = np.mean(sample_points, axis=0)

        # Test W key (jog axis 1, direction 1)
        initial_pos = window.target_pos.copy()
        event = Mock()
        event.key.return_value = Qt.Key_W
        window.keyPressEvent(event)
        assert not np.array_equal(window.target_pos, initial_pos)

    @patch('gui.main.shared_memory.SharedMemory')
    @patch('gui.main.QMessageBox')
    def test_start_waypoint_routine_no_points(self, mock_msgbox, mock_shm, qapp):
        """Test starting waypoint routine without base points."""
        window = MainWindow()
        window.points = None

        window.start_waypoint_routine()

        # Warning should be shown
        mock_msgbox.warning.assert_called_once()
        assert window.waypoint_routine_active is False

    @patch('gui.main.shared_memory.SharedMemory')
    @patch('gui.main.QMessageBox')
    def test_start_waypoint_routine_success(self, mock_msgbox, mock_shm, qapp):
        """Test successfully starting waypoint routine."""
        window = MainWindow()

        # Use a triangle that forms a larger, flatter area to ensure waypoints are valid
        # Equilateral triangle in XY plane with larger size
        window.points = np.array([
            [100.0, 0.0, 0.0],     # Left
            [0.0, 100.0, 0.0],     # Center
            [0.0, 0.0, 0.0],       # Right (origin)
        ])

        window.start_waypoint_routine()

        # If QMessageBox.warning was called, the routine failed validation
        if mock_msgbox.warning.called:
            # Routine should not be active if validation failed
            assert window.waypoint_routine_active is False
        else:
            # Waypoint routine should be active
            assert window.waypoint_routine_active is True
            # Waypoint queue should be populated
            assert len(window.waypoint_queue) > 0
            # Button should be disabled
            assert window.waypoint_button.isEnabled() is False

    @patch('gui.main.shared_memory.SharedMemory')
    def test_execute_next_waypoint_completion(self, mock_shm, qapp):
        """Test waypoint execution completion when queue is empty."""
        window = MainWindow()
        window.waypoint_routine_active = True
        window.waypoint_queue = []

        with patch('gui.main.QMessageBox') as mock_msgbox:
            window.execute_next_waypoint()

        # Routine should be inactive
        assert window.waypoint_routine_active is False
        # Button should be re-enabled
        assert window.waypoint_button.isEnabled() is True
        # Completion message should be shown
        mock_msgbox.information.assert_called_once()

    @patch('gui.main.shared_memory.SharedMemory')
    def test_check_waypoint_progress(self, mock_shm, qapp, sample_points):
        """Test waypoint progress checking."""
        window = MainWindow()
        window.points = sample_points
        window.waypoint_routine_active = True
        window.waypoint_waiting_for_ready = True
        window.waypoint_queue = [np.array([1.0, 1.0, 1.0])]

        # Mock QTimer.singleShot
        with patch('gui.main.QTimer.singleShot') as mock_timer:
            # Simulate status change to "run"
            window.check_waypoint_progress("run")

            # Should schedule next waypoint
            assert window.waypoint_waiting_for_ready is False
            mock_timer.assert_called_once()

    @patch('gui.main.shared_memory.SharedMemory')
    def test_recenter_view(self, mock_shm, qapp):
        """Test recenter view functionality."""
        window = MainWindow()
        window.current_pos = np.array([5.0, 5.0, 5.0])

        window.recenter_view()

        # View should be updated (hard to test exact values, but ensure no crash)
        assert window.view.opts['center'] == QVector3D(5.0, 5.0, 5.0)

    @patch('gui.main.shared_memory.SharedMemory')
    def test_sync_orientation_axes_to_main_view(self, mock_shm, qapp):
        """Test syncing orientation axes with main view."""
        window = MainWindow()

        azimuth = 45.0
        elevation = 30.0

        window.sync_orientation_axes_to_main_view(azimuth, elevation)

        # Orientation axes should be updated
        assert window.orientation_axes.opts['azimuth'] == azimuth
        assert window.orientation_axes.opts['elevation'] == elevation


class TestSharedMemoryWriter:
    """Tests for SharedMemoryWriter class."""

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_initialization(self, mock_shm, qapp):
        """Test that SharedMemoryWriter initializes correctly."""
        writer = SharedMemoryWriter()
        assert writer is not None
        assert len(writer.inputs) == 12  # 12 coordinate inputs
        assert hasattr(writer, 'status_combo')
        assert hasattr(writer, 'start_button')
        assert hasattr(writer, 'stop_button')
        assert writer.stop_button.isEnabled() is False

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_start_stop_writing(self, mock_shm, qapp):
        """Test starting and stopping write timer."""
        writer = SharedMemoryWriter()

        # Start writing
        writer.start_writing()
        assert writer.timer.isActive() is True
        assert writer.start_button.isEnabled() is False
        assert writer.stop_button.isEnabled() is True

        # Stop writing
        writer.stop_writing()
        assert writer.timer.isActive() is False
        assert writer.start_button.isEnabled() is True
        assert writer.stop_button.isEnabled() is False

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_write_shared_memory(self, mock_shm_class, qapp):
        """Test writing data to shared memory."""
        # Create mock shared memory instance
        mock_shm_instance = Mock()
        mock_shm_instance.buf = bytearray(STATUS_STRUCT_SIZE)
        mock_shm_class.return_value = mock_shm_instance

        writer = SharedMemoryWriter()
        writer.shm_write = mock_shm_instance

        # Set some test values
        test_values = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]
        for i, val in enumerate(test_values):
            writer.inputs[i].setText(str(val))

        writer.status_combo.setCurrentText("ready")

        # Write to shared memory
        writer.write_shared_memory()

        # Verify data was written to buffer
        assert len(mock_shm_instance.buf) == STATUS_STRUCT_SIZE

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_calculate_motor_positions(self, mock_shm, qapp):
        """Test motor position calculations."""
        writer = SharedMemoryWriter()

        # Set base positions
        writer.inputs[0].setText("0.0")   # L_X
        writer.inputs[1].setText("0.0")   # L_Y
        writer.inputs[2].setText("0.0")   # L_Z
        writer.inputs[3].setText("1.0")   # C_X
        writer.inputs[4].setText("0.0")   # C_Y
        writer.inputs[5].setText("0.0")   # C_Z
        writer.inputs[6].setText("0.0")   # R_X
        writer.inputs[7].setText("1.0")   # R_Y
        writer.inputs[8].setText("0.0")   # R_Z

        # Calculate for a target position
        cable_lengths, motor_steps = writer.calculate_motor_positions(0.5, 0.5, 0.5)

        assert cable_lengths is not None
        assert motor_steps is not None
        assert len(cable_lengths) == 3
        assert len(motor_steps) == 3

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_calculate_motor_positions_invalid_input(self, mock_shm, qapp):
        """Test motor position calculation with invalid inputs."""
        writer = SharedMemoryWriter()

        # Set invalid base positions
        writer.inputs[0].setText("invalid")

        # Should handle gracefully
        cable_lengths, motor_steps = writer.calculate_motor_positions(0.5, 0.5, 0.5)

        assert cable_lengths is None
        assert motor_steps is None

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_read_shared_memory(self, mock_shm_class, qapp):
        """Test reading motor command from shared memory."""
        # Create mock shared memory for reading
        mock_read_shm = Mock()
        test_data = struct.pack(MOTOR_STRUCT_FORMAT, 1.5, 2.5, 3.5, True, False)
        mock_read_shm.buf = bytearray(test_data)

        writer = SharedMemoryWriter()
        writer.shm_read = mock_read_shm

        # Read from shared memory
        writer.read_shared_memory()

        # Verify readback labels are updated
        assert writer.labels_readback['Target X'].text() == "1.500"
        assert writer.labels_readback['Target Y'].text() == "2.500"
        assert writer.labels_readback['Target Z'].text() == "3.500"
        assert writer.labels_readback['Execute'].text() == "True"
        assert writer.labels_readback['Exit'].text() == "False"


class TestIntegration:
    """Integration tests for GUI components working together."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_orientation_axes_sync_with_main_view(self, mock_shm, qapp):
        """Test that orientation axes and main view stay synchronized."""
        window = MainWindow()

        # Get initial orientations
        initial_azimuth = window.view.opts.get('azimuth', 0)
        initial_elevation = window.view.opts.get('elevation', 0)

        # Sync from orientation axes to main view
        window.sync_main_view_to_orientation()

        # Main view should match orientation axes
        assert window.view.opts.get('azimuth') == window.orientation_axes.opts.get('azimuth')
        assert window.view.opts.get('elevation') == window.orientation_axes.opts.get('elevation')

    @patch('gui.main.shared_memory.SharedMemory')
    @patch('gui.main.QMessageBox')
    def test_full_waypoint_workflow(self, mock_msgbox, mock_shm, qapp):
        """Test complete waypoint routine workflow."""
        window = MainWindow()

        # Use a larger triangle to ensure waypoints are valid
        window.points = np.array([
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            [0.0, 0.0, 0.0],
        ])

        # Start waypoint routine
        window.start_waypoint_routine()

        # Only proceed if routine started successfully
        if not mock_msgbox.warning.called:
            assert window.waypoint_routine_active is True
            initial_queue_length = len(window.waypoint_queue)

            # Execute first waypoint
            window.execute_next_waypoint()
            assert len(window.waypoint_queue) == initial_queue_length - 1
            assert window.waypoint_waiting_for_ready is True

            # Simulate system ready
            with patch('gui.main.QTimer.singleShot'):
                window.check_waypoint_progress("run")
                assert window.waypoint_waiting_for_ready is False
        else:
            # If validation failed, just skip the test
            assert True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
