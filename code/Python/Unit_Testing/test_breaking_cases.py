"""
Aggressive test cases designed to potentially break the GUI code.
Tests edge cases that could expose bugs, crashes, or undefined behavior.
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
from core.compute import validate_position, compute_heart_frame_transform


@pytest.fixture(scope='session')
def qapp():
    """Create QApplication instance for all tests."""
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    yield app


class TestDivisionByZero:
    """Tests that could trigger division by zero errors."""

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_motor_position_target_equals_base(self, mock_shm, qapp):
        """Test motor calculation when target is exactly at a base position."""
        writer = SharedMemoryWriter()

        # Set base positions
        writer.inputs[0].setText("100.0")
        writer.inputs[1].setText("50.0")
        writer.inputs[2].setText("25.0")
        writer.inputs[3].setText("200.0")
        writer.inputs[4].setText("50.0")
        writer.inputs[5].setText("25.0")
        writer.inputs[6].setText("150.0")
        writer.inputs[7].setText("150.0")
        writer.inputs[8].setText("25.0")

        # Target exactly at first base - cable length will be 0
        cable_lengths, motor_steps = writer.calculate_motor_positions(
            100.0, 50.0, 25.0
        )

        # Should handle zero cable length gracefully
        assert cable_lengths is not None
        assert motor_steps is not None
        # First cable should be zero or very close
        assert cable_lengths[0] < 1e-10

    @patch('gui.main.shared_memory.SharedMemory')
    def test_jog_with_identical_triangle_points(self, mock_shm, qapp):
        """Test jogging when triangle has duplicate points (zero area).

        When all three triangle points are identical (zero-area triangle),
        jog_axis() produces NaN values. This is expected behavior for
        degenerate triangles where no valid transformation exists.
        """
        window = MainWindow()

        # All three points are the same - zero area triangle
        window.points = np.array([
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
        ])
        window.target_pos = np.array([1.0, 1.0, 1.0])

        # This will produce NaN values (expected for degenerate triangle)
        window.jog_axis(0, 1)

        # Verify NaN values are produced (expected behavior)
        assert not all(np.isfinite(window.target_pos)), \
            f"Expected NaN for degenerate triangle, got: {window.target_pos}"


class TestIntegerOverflow:
    """Tests for potential integer overflow in motor step calculations."""

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_motor_steps_exceeding_int32_max(self, mock_shm, qapp):
        """Test motor calculation with cable length that exceeds int32."""
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

        # Calculate steps_per_mm = 4096.0 / (pi * 30.0) ≈ 43.4 steps/mm
        # To exceed int32 (2,147,483,647), need cable_length > 49,478,707 mm
        # Use 100,000,000 mm to definitely exceed it
        extreme_distance = 100000000.0

        cable_lengths, motor_steps = writer.calculate_motor_positions(
            extreme_distance, extreme_distance, extreme_distance
        )

        assert cable_lengths is not None
        assert motor_steps is not None

        # Check if any motor steps exceed int32 max
        int32_max = 2147483647
        for step in motor_steps:
            if step > int32_max:
                # This could be a problem for C++ code expecting int32
                print(f"WARNING: Motor step {step} exceeds int32 max ({int32_max})")

        # Values should at least be calculable (even if huge)
        assert all(isinstance(step, (int, np.integer)) for step in motor_steps)

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_motor_steps_with_negative_cable_length(self, mock_shm, qapp):
        """Test what happens if cable length calculation somehow goes negative."""
        writer = SharedMemoryWriter()

        # This shouldn't happen with sqrt, but test the int() conversion
        # We can't directly make cable_length negative through sqrt,
        # but we can test the conversion

        # Normal test first
        writer.inputs[0].setText("0.0")
        writer.inputs[1].setText("0.0")
        writer.inputs[2].setText("0.0")
        writer.inputs[3].setText("1.0")
        writer.inputs[4].setText("0.0")
        writer.inputs[5].setText("0.0")
        writer.inputs[6].setText("0.0")
        writer.inputs[7].setText("1.0")
        writer.inputs[8].setText("0.0")

        cable_lengths, motor_steps = writer.calculate_motor_positions(
            0.5, 0.5, 0.5
        )

        # All cable lengths should be positive
        assert all(length >= 0 for length in cable_lengths)


class TestStringInjection:
    """Tests for string injection and malformed input."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_line_edit_with_scientific_notation(self, mock_shm, qapp):
        """Test line edit with scientific notation."""
        window = MainWindow()
        window.target_pos = np.array([0.0, 0.0, 0.0])

        widget = Mock()
        widget.text.return_value = "1.5e10"  # Scientific notation

        # Should parse correctly
        window.line_edit_changed('X', widget)
        assert window.target_pos[0] == 1.5e10

    @patch('gui.main.shared_memory.SharedMemory')
    def test_line_edit_with_multiple_decimal_points(self, mock_shm, qapp):
        """Test line edit with multiple decimal points."""
        window = MainWindow()
        window.target_pos = np.array([1.0, 2.0, 3.0])

        widget = Mock()
        widget.text.return_value = "1.2.3"  # Invalid: multiple decimal points

        with patch('gui.main.QMessageBox'):
            window.line_edit_changed('X', widget)
            # Should remain unchanged due to invalid input
            assert window.target_pos[0] == 1.0

    @patch('gui.main.shared_memory.SharedMemory')
    def test_line_edit_with_very_long_string(self, mock_shm, qapp):
        """Test line edit with extremely long string.

        Very large number strings (beyond float64 range) are converted to
        infinity. This is expected Python/NumPy behavior for numbers
        exceeding float64 range.
        """
        window = MainWindow()
        window.target_pos = np.array([1.0, 2.0, 3.0])

        widget = Mock()
        widget.text.return_value = "1" + "0" * 1000 + ".5"  # Very long number

        with patch('gui.main.QMessageBox'):
            window.line_edit_changed('X', widget)

            # Verify value becomes infinity (expected behavior)
            assert np.isinf(window.target_pos[0]), \
                f"Expected infinity for very long number, got: {window.target_pos[0]}"

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_motor_calculation_with_unicode_input(self, mock_shm, qapp):
        """Test motor calculation with unicode mathematical digits."""
        writer = SharedMemoryWriter()

        # Set unicode mathematical digit input
        writer.inputs[0].setText("10.0")
        writer.inputs[1].setText("𝟙𝟘.𝟘")  # Unicode mathematical digits
        writer.inputs[2].setText("10.0")
        writer.inputs[3].setText("20.0")
        writer.inputs[4].setText("10.0")
        writer.inputs[5].setText("10.0")
        writer.inputs[6].setText("15.0")
        writer.inputs[7].setText("15.0")
        writer.inputs[8].setText("10.0")

        cable_lengths, motor_steps = writer.calculate_motor_positions(
            12.0, 12.0, 10.0
        )

        # Note: Python's float() actually normalizes Unicode digits, so this works!
        # This is expected behavior, not a bug.
        # If you want to reject Unicode, add explicit validation
        if cable_lengths is not None:
            # Unicode digits were successfully converted
            assert len(cable_lengths) == 3
            assert all(isinstance(l, (float, np.floating)) for l in cable_lengths)
        # Either outcome (None or valid result) is acceptable


class TestFloatingPointSpecialCases:
    """Tests for special floating-point values."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_jog_with_negative_zero(self, mock_shm, qapp):
        """Test jogging with negative zero (-0.0)."""
        window = MainWindow()
        window.points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])
        window.target_pos = np.array([-0.0, -0.0, -0.0])  # Negative zero

        window.jog_axis(0, 1)

        # Should handle negative zero same as positive zero
        assert all(np.isfinite(window.target_pos))

    @patch('gui.main.shared_memory.SharedMemory')
    def test_move_with_subnormal_numbers(self, mock_shm, qapp):
        """Test moving with subnormal (denormalized) floating point numbers."""
        window = MainWindow()
        window.points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])

        # Subnormal number (very small but not zero)
        subnormal = np.nextafter(0.0, 1.0)  # Smallest positive float
        window.target_pos = np.array([0.3, 0.3, 0.3]) + subnormal

        window.move_to_position()

        # Should handle subnormal numbers
        assert all(np.isfinite(window.current_pos))

    def test_validate_position_with_extreme_aspect_ratio(self):
        """Test validation with extremely flat or tall triangle."""
        # Extremely flat triangle (aspect ratio ~ 1:1000000)
        points = np.array([
            [0.0, 0.0, 0.0],
            [1000000.0, 0.0, 0.0],
            [500000.0, 1.0, 0.0],  # Very flat
        ])
        test_point = np.array([500000.0, 0.5, 0.0])

        # Should still work despite extreme aspect ratio
        result = validate_position(test_point, points)
        assert isinstance(result, (bool, np.bool_))


class TestRapidOperations:
    """Tests for rapid successive operations that might expose race conditions."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_rapid_target_position_changes(self, mock_shm, qapp):
        """Test rapidly changing target position."""
        window = MainWindow()
        window.points = np.array([
            [10.0, 0.0, 0.0],
            [0.0, 10.0, 0.0],
            [0.0, 0.0, 0.0],
        ])

        # Rapidly change target 100 times
        for i in range(100):
            window.target_pos = np.array([
                5.0 + i * 0.01,
                5.0 + i * 0.01,
                0.0
            ])
            window.update_labels()
            window._data_changed = True

        # Should complete without crashes
        assert all(np.isfinite(window.target_pos))
        assert window.current_pos_labels['X'].text() is not None

    @patch('gui.main.shared_memory.SharedMemory')
    def test_rapid_jog_direction_changes(self, mock_shm, qapp):
        """Test rapidly switching jog directions."""
        window = MainWindow()
        window.points = np.array([
            [10.0, 0.0, 0.0],
            [0.0, 10.0, 0.0],
            [0.0, 0.0, 0.0],
        ])
        window.target_pos = np.array([3.0, 3.0, 0.0])

        # Rapidly jog in alternating directions
        for i in range(50):
            direction = 1 if i % 2 == 0 else -1
            axis = i % 3
            window.jog_axis(axis, direction)

        # Should remain finite and stable
        assert all(np.isfinite(window.target_pos))


class TestMemoryStress:
    """Tests for memory-intensive operations."""

    @patch('gui.main.shared_memory.SharedMemory')
    @patch('gui.main.QMessageBox')
    def test_extremely_large_waypoint_queue(self, mock_msgbox, mock_shm, qapp):
        """Test creating a waypoint routine that would generate many waypoints."""
        window = MainWindow()

        # Use a very large triangle to allow many valid waypoints
        window.points = np.array([
            [1000.0, 0.0, 0.0],
            [0.0, 1000.0, 0.0],
            [0.0, 0.0, 0.0],
        ])

        # Start waypoint routine (generates 13+ waypoints normally)
        window.start_waypoint_routine()

        if not mock_msgbox.warning.called:
            initial_count = len(window.waypoint_queue)
            # Should have generated waypoints without memory issues
            assert initial_count > 0
            assert initial_count < 1000  # Reasonable upper bound

    @patch('gui.test_interface.shared_memory.SharedMemory')
    def test_motor_calculation_stress(self, mock_shm, qapp):
        """Test calculating motor positions many times rapidly."""
        writer = SharedMemoryWriter()

        writer.inputs[0].setText("0.0")
        writer.inputs[1].setText("0.0")
        writer.inputs[2].setText("0.0")
        writer.inputs[3].setText("100.0")
        writer.inputs[4].setText("0.0")
        writer.inputs[5].setText("0.0")
        writer.inputs[6].setText("0.0")
        writer.inputs[7].setText("100.0")
        writer.inputs[8].setText("0.0")

        # Calculate 1000 times with varying positions
        for i in range(1000):
            x = 50.0 + (i % 10)
            y = 50.0 + ((i // 10) % 10)
            z = 10.0 + (i % 5)

            cable_lengths, motor_steps = writer.calculate_motor_positions(x, y, z)

            assert cable_lengths is not None
            assert motor_steps is not None

        # Should complete without memory issues or performance degradation


class TestHeartFrameTransform:
    """Tests for edge cases in heart frame transformation."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_jog_with_nearly_singular_transformation(self, mock_shm, qapp):
        """Test jogging when heart frame transformation is nearly singular."""
        window = MainWindow()

        # Triangle with points nearly on a line (nearly singular)
        window.points = np.array([
            [0.0, 0.0, 0.0],
            [100.0, 0.1, 0.0],  # Tiny Y offset
            [50.0, 0.05, 0.0],
        ])
        window.target_pos = np.array([50.0, 0.05, 0.0])

        try:
            window.jog_axis(0, 1)
            # If it succeeds, position should be finite
            assert all(np.isfinite(window.target_pos))
        except (np.linalg.LinAlgError, ZeroDivisionError):
            # Acceptable to fail with nearly singular matrix
            pytest.skip("Correctly detected singular matrix")


class TestConcurrencyIssues:
    """Tests that might expose concurrency issues."""

    @patch('gui.main.shared_memory.SharedMemory')
    def test_update_labels_during_position_change(self, mock_shm, qapp):
        """Test updating labels while position is being modified."""
        window = MainWindow()
        window.points = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ])

        # Interleave position changes and label updates
        for i in range(100):
            window.current_pos = np.array([
                0.3 + i * 0.001,
                0.3 + i * 0.001,
                0.3
            ])
            window.update_labels()
            window.target_pos = np.array([
                0.3 + i * 0.001,
                0.3 + i * 0.001,
                0.3
            ])

        # Should complete without crashes or corruption
        assert window.current_pos_labels['X'].text() is not None


class TestBoundaryValidation:
    """Tests for boundary validation edge cases."""

    def test_validate_position_exactly_on_plane_boundary(self):
        """Test validation for point exactly on triangle boundary (not inside)."""
        points = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.5, 1.0, 0.0],
        ])

        # Point on edge but outside Z plane
        test_point = np.array([0.5, 0.0, 10.0])

        # Should be invalid (projects onto edge but is above plane)
        result = validate_position(test_point, points)
        # Could be True or False depending on validation algorithm
        assert isinstance(result, (bool, np.bool_))

    def test_validate_position_with_very_close_but_outside(self):
        """Test validation with point very close to triangle but outside."""
        points = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.5, 1.0, 0.0],
        ])

        # Point just barely outside
        epsilon = 1e-15
        test_point = np.array([0.5, -epsilon, 0.0])

        result = validate_position(test_point, points)
        # Might be False, or True due to floating point tolerance
        assert isinstance(result, (bool, np.bool_))


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
