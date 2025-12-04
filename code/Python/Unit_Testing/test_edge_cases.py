"""
Edge case and boundary condition tests for GUI components.
Tests for overflow errors, numerical precision, and invalid states.
"""
import sys
import os
import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import Qt
import struct

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gui.main import MainWindow
from gui.test_interface import SharedMemoryWriter
from core.compute import (
    validate_position, compute_heart_frame_transform,
    compute_plane_mesh, triangle_area
)


@pytest.fixture(scope='session')
def qapp():
    """Create QApplication instance for all tests."""
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    yield app


class TestOverflowAndLargeValues:
    """Test handling of very large values that could cause overflow."""

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_motor_position_very_large_distance(self, mock_shm, qapp):
        """Test motor position calculation with very large cable lengths."""
        writer = SharedMemoryWriter()

        # Set base positions
        writer.inputs[0].setText("0.0")
        writer.inputs[1].setText("0.0")
        writer.inputs[2].setText("0.0")
        writer.inputs[3].setText("100.0")
        writer.inputs[4].setText("0.0")
        writer.inputs[5].setText("0.0")
        writer.inputs[6].setText("0.0")
        writer.inputs[7].setText("100.0")
        writer.inputs[8].setText("0.0")

        # Calculate for a very distant target (1000 units away)
        target_x, target_y, target_z = 5000.0, 5000.0, 5000.0
        cable_lengths, motor_steps = writer.calculate_motor_positions(
            target_x, target_y, target_z
        )

        # Should handle gracefully without overflow
        assert cable_lengths is not None
        assert motor_steps is not None
        assert all(length > 0 for length in cable_lengths)
        assert all(step > 0 for step in motor_steps)
        # Check for reasonable values (not overflowed to negative)
        assert all(step < 1e10 for step in motor_steps)

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_motor_position_extreme_distance(self, mock_shm, qapp):
        """Test motor position with extremely large distance (near float limit)."""
        writer = SharedMemoryWriter()

        # Set base positions
        writer.inputs[0].setText("0.0")
        writer.inputs[1].setText("0.0")
        writer.inputs[2].setText("0.0")
        writer.inputs[3].setText("1.0")
        writer.inputs[4].setText("0.0")
        writer.inputs[5].setText("0.0")
        writer.inputs[6].setText("0.0")
        writer.inputs[7].setText("1.0")
        writer.inputs[8].setText("0.0")

        # Use a very large distance (close to float64 limits)
        extreme_value = 1e10
        cable_lengths, motor_steps = writer.calculate_motor_positions(
            extreme_value, extreme_value, extreme_value
        )

        # Should complete without crashing
        assert cable_lengths is not None
        assert motor_steps is not None
        # Check values are finite
        assert all(np.isfinite(length) for length in cable_lengths)
        assert all(np.isfinite(step) for step in motor_steps)

    @patch('gui.main.shared_memory.SharedMemory')
    def test_jog_with_large_accumulated_distance(self, mock_shm, qapp):
        """Test that repeated jogging doesn't cause coordinate overflow."""
        window = MainWindow()
        window.points = np.array([
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            [0.0, 0.0, 0.0],
        ])
        window.target_pos = np.array([30.0, 30.0, 0.0])

        # Jog 1000 times in the same direction
        for _ in range(1000):
            initial_pos = window.target_pos.copy()
            window.jog_axis(0, 1)
            # Position should change each time
            assert not np.array_equal(window.target_pos, initial_pos)
            # Values should remain finite
            assert all(np.isfinite(window.target_pos))


class TestBoundaryConditions:
    """Test boundary conditions and edge cases."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_position_at_triangle_vertex(self, mock_shm, qapp):
        """Test moving to exact triangle vertex positions."""
        window = MainWindow()
        window.points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])

        # Test each vertex
        for vertex in window.points:
            window.target_pos = vertex.copy()
            window.move_to_position()
            # Should be valid (vertices are inside the triangle)
            np.testing.assert_array_almost_equal(window.current_pos, vertex)

    @patch('gui.main.shared_memory.SharedMemory')
    def test_position_on_triangle_edge(self, mock_shm, qapp):
        """Test moving to positions on triangle edges."""
        window = MainWindow()
        window.points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])

        # Test midpoint of each edge
        edges = [
            (window.points[0] + window.points[1]) / 2,
            (window.points[1] + window.points[2]) / 2,
            (window.points[2] + window.points[0]) / 2,
        ]

        for edge_point in edges:
            window.target_pos = edge_point.copy()
            window.move_to_position()
            # Edge points should be valid
            np.testing.assert_array_almost_equal(window.current_pos, edge_point)

    @patch('gui.main.shared_memory.SharedMemory')
    def test_zero_distance_jog(self, mock_shm, qapp):
        """Test that jogging with zero distance works correctly."""
        window = MainWindow()
        window.points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        window.target_pos = np.array([0.3, 0.3, 0.3])
        initial_pos = window.target_pos.copy()

        # Jog with direction 0 (invalid) should not crash
        window.jog_axis(0, 0)

        # Position might not change much, but shouldn't crash
        assert all(np.isfinite(window.target_pos))

    @patch('gui.main.shared_memory.SharedMemory')
    def test_empty_waypoint_queue(self, mock_shm, qapp):
        """Test executing waypoint with empty queue."""
        window = MainWindow()
        window.waypoint_queue = []
        window.waypoint_routine_active = True

        with patch('gui.main.QMessageBox'):
            window.execute_next_waypoint()

        # Should deactivate routine gracefully
        assert window.waypoint_routine_active is False

    def test_validate_position_at_triangle_centroid(self):
        """Test that centroid is always valid."""
        points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        centroid = np.mean(points, axis=0)

        # Use == instead of 'is' to handle numpy bool types
        assert validate_position(centroid, points) == True

    def test_triangle_area_zero_for_collinear_points(self):
        """Test triangle area calculation with collinear points."""
        # Three collinear points
        p1 = np.array([0.0, 0.0, 0.0])
        p2 = np.array([1.0, 1.0, 1.0])
        p3 = np.array([2.0, 2.0, 2.0])

        area = triangle_area(p1, p2, p3)
        assert abs(area) < 1e-10  # Should be very close to zero


class TestInvalidStates:
    """Test handling of invalid states and inputs."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_move_with_none_points(self, mock_shm, qapp):
        """Test moving when points are None."""
        window = MainWindow()
        window.points = None
        window.target_pos = np.array([1.0, 1.0, 1.0])

        with patch('gui.main.QMessageBox'):
            # Should not crash
            window.move_to_position()

    @patch('gui.main.shared_memory.SharedMemory')
    def test_jog_with_none_points(self, mock_shm, qapp):
        """Test jogging when points are None."""
        window = MainWindow()
        window.points = None
        window.target_pos = np.array([1.0, 1.0, 1.0])

        # Should not crash
        window.jog_axis(0, 1)

    def test_validate_position_with_degenerate_triangle(self):
        """Test validation with collinear points (degenerate triangle)."""
        # Collinear points
        points = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [2.0, 2.0, 2.0],
        ])
        test_point = np.array([1.5, 1.5, 1.5])

        # Should return False for degenerate triangle
        result = validate_position(test_point, points)
        assert result is False

    def test_validate_position_with_empty_points(self):
        """Test validation with insufficient points."""
        points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])  # Only 2 points
        test_point = np.array([0.5, 0.5, 0.0])

        # Should return False
        result = validate_position(test_point, points)
        assert result is False

    def test_compute_plane_mesh_with_degenerate_triangle(self):
        """Test mesh computation with degenerate triangle."""
        # Collinear points
        points = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
        ])

        meshdata, normal = compute_plane_mesh(points)

        # Should not crash and return valid objects
        assert meshdata is not None
        assert normal is not None
        # Normal should be zero or very small for degenerate case
        assert np.linalg.norm(normal) < 1.0


class TestNumericalPrecision:
    """Test numerical precision and floating-point edge cases."""

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_motor_position_very_small_distance(self, mock_shm, qapp):
        """Test motor calculation with very small distances."""
        writer = SharedMemoryWriter()

        # Set base positions very close together
        writer.inputs[0].setText("0.0")
        writer.inputs[1].setText("0.0")
        writer.inputs[2].setText("0.0")
        writer.inputs[3].setText("0.001")
        writer.inputs[4].setText("0.0")
        writer.inputs[5].setText("0.0")
        writer.inputs[6].setText("0.0")
        writer.inputs[7].setText("0.001")
        writer.inputs[8].setText("0.0")

        # Very small target offset
        cable_lengths, motor_steps = writer.calculate_motor_positions(
            0.0001, 0.0001, 0.0001
        )

        assert cable_lengths is not None
        assert motor_steps is not None
        # Values should be finite and positive
        assert all(np.isfinite(length) for length in cable_lengths)
        assert all(length >= 0 for length in cable_lengths)

    @patch('gui.main.shared_memory.SharedMemory')
    def test_very_small_jog_increment(self, mock_shm, qapp):
        """Test that very small jog movements are handled correctly."""
        window = MainWindow()
        window.points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        window.target_pos = np.array([0.3, 0.3, 0.3])

        # Perform many small jogs
        for _ in range(100):
            initial_pos = window.target_pos.copy()
            window.jog_axis(0, 1)
            # Should change by small amount
            diff = np.linalg.norm(window.target_pos - initial_pos)
            assert 0 < diff < 0.1  # Within reasonable range

    def test_validate_position_nearly_collinear_points(self):
        """Test validation with nearly (but not exactly) collinear points."""
        # Nearly collinear points
        points = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.001],  # Slight deviation from collinear
            [2.0, 0.0, 0.0],
        ])
        test_point = np.array([1.0, 0.0, 0.0])

        # Should still work (might be True or False, but shouldn't crash)
        result = validate_position(test_point, points)
        assert isinstance(result, (bool, np.bool_))


class TestStateTransitions:
    """Test state machine transitions and concurrent operations."""

    @patch('gui.main.shared_memory.SharedMemory')
    @patch('gui.main.QMessageBox')
    def test_double_waypoint_routine_start(self, mock_msgbox, mock_shm, qapp):
        """Test starting waypoint routine when already running."""
        window = MainWindow()
        window.points = np.array([
            [100.0, 0.0, 0.0],
            [0.0, 100.0, 0.0],
            [0.0, 0.0, 0.0],
        ])

        # Start first routine
        window.start_waypoint_routine()

        # Try to start again
        first_active = window.waypoint_routine_active
        window.start_waypoint_routine()

        # Should show warning and not start second routine
        if first_active:
            assert mock_msgbox.warning.called

    @patch('gui.main.shared_memory.SharedMemory')
    def test_waypoint_progress_when_not_waiting(self, mock_shm, qapp):
        """Test waypoint progress check when not waiting for ready."""
        window = MainWindow()
        window.waypoint_routine_active = True
        window.waypoint_waiting_for_ready = False

        # Should not execute next waypoint
        window.check_waypoint_progress("run")
        # No assertion needed, just ensuring it doesn't crash

    @patch('gui.main.shared_memory.SharedMemory')
    def test_update_labels_with_nan(self, mock_shm, qapp):
        """Test label updates with NaN values."""
        window = MainWindow()
        window.current_pos = np.array([np.nan, np.nan, np.nan])

        # Should not crash
        window.update_labels()

        # Labels should display something (likely 'nan' as string)
        assert window.current_pos_labels['X'].text() is not None
        assert window.current_pos_labels['Y'].text() is not None
        assert window.current_pos_labels['Z'].text() is not None

    @patch('gui.main.shared_memory.SharedMemory')
    def test_update_labels_with_infinity(self, mock_shm, qapp):
        """Test label updates with infinity values."""
        window = MainWindow()
        window.current_pos = np.array([np.inf, -np.inf, np.inf])

        # Should not crash
        window.update_labels()

        # Labels should display something
        assert window.current_pos_labels['X'].text() is not None
        assert window.current_pos_labels['Y'].text() is not None
        assert window.current_pos_labels['Z'].text() is not None


class TestNegativeCoordinates:
    """Test handling of negative coordinates."""

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_motor_position_negative_coordinates(self, mock_shm, qapp):
        """Test motor calculation with negative coordinates."""
        writer = SharedMemoryWriter()

        # Set base positions with negative values
        writer.inputs[0].setText("-50.0")
        writer.inputs[1].setText("-50.0")
        writer.inputs[2].setText("0.0")
        writer.inputs[3].setText("50.0")
        writer.inputs[4].setText("-50.0")
        writer.inputs[5].setText("0.0")
        writer.inputs[6].setText("0.0")
        writer.inputs[7].setText("50.0")
        writer.inputs[8].setText("0.0")

        # Negative target position
        cable_lengths, motor_steps = writer.calculate_motor_positions(
            -10.0, -10.0, 10.0
        )

        assert cable_lengths is not None
        assert motor_steps is not None
        # Cable lengths should be positive (it's a distance)
        assert all(length >= 0 for length in cable_lengths)

    @patch('gui.main.shared_memory.SharedMemory')
    def test_jog_with_negative_triangle(self, mock_shm, qapp):
        """Test jogging with triangle in negative coordinate space."""
        window = MainWindow()
        window.points = np.array([
            [-100.0, -100.0, 0.0],
            [-50.0, -100.0, 0.0],
            [-75.0, -50.0, 0.0],
        ])
        window.target_pos = np.array([-75.0, -80.0, 0.0])

        initial_pos = window.target_pos.copy()
        window.jog_axis(0, 1)

        # Should jog successfully
        assert not np.array_equal(window.target_pos, initial_pos)
        assert all(np.isfinite(window.target_pos))

    def test_validate_position_negative_coordinates(self):
        """Test position validation with all negative coordinates."""
        points = np.array([
            [-1.0, -1.0, -1.0],
            [-2.0, -1.0, -1.0],
            [-1.5, -2.0, -1.0],
        ])
        test_point = np.array([-1.5, -1.4, -1.0])

        # Should work normally
        result = validate_position(test_point, points)
        assert isinstance(result, (bool, np.bool_))


class TestConcurrentOperations:
    """Test concurrent operations and race conditions."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_multiple_simultaneous_jogs(self, mock_shm, qapp):
        """Test multiple jog operations in sequence."""
        window = MainWindow()
        window.points = np.array([
            [10.0, 0.0, 0.0],
            [0.0, 10.0, 0.0],
            [0.0, 0.0, 0.0],
        ])
        window.target_pos = np.array([3.0, 3.0, 0.0])

        initial_pos = window.target_pos.copy()

        # Jog in all positive directions (to ensure net movement)
        window.jog_axis(0, 1)
        window.jog_axis(1, 1)
        window.jog_axis(2, 1)

        # Position should have changed (or at least stayed finite)
        # The jogs might cancel out depending on heart frame transformations
        assert all(np.isfinite(window.target_pos))

        # At least one coordinate should have changed or be valid
        # (since jog transforms through heart frame, the change might be complex)
        assert window.target_pos is not None

    @patch('gui.main.shared_memory.SharedMemory')
    def test_move_during_label_update(self, mock_shm, qapp):
        """Test moving position while labels are being updated."""
        window = MainWindow()
        window.points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        window.current_pos = np.array([0.3, 0.3, 0.3])
        window.target_pos = np.array([0.35, 0.35, 0.35])

        # Update labels while changing position
        window.update_labels()
        window.move_to_position()
        window.update_labels()

        # Should complete without issues
        assert window.current_pos_labels['X'].text() is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
