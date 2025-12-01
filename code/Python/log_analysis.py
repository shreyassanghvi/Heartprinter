import re
from datetime import datetime
from collections import defaultdict, Counter
import sys
import glob
import os
import csv
import matplotlib.pyplot as plt
import matplotlib.dates as mdates

def is_valid_log_file(file_path):
    """
    Check if file is a valid Heart Printer log file
    """
    try:
        with open(file_path, 'r') as f:
            # Read first few lines to check for log initialization marker
            for i, line in enumerate(f):
                if i > 10:  # Only check first 10 lines
                    break
                if 'Logger initialized with both file and console sinks' in line:
                    return True
        return False
    except Exception as e:
        print(f"Error checking file {file_path}: {e}")
        return False

def analyze_heart_printer_logs(log_content, log_file_path=None):
    """
    Analyze Heart Printer System logs and provide a comprehensive summary
    Returns both the report text and the summary data structure for CSV generation
    """
    lines = log_content.strip().split('\n')

    # Initialize data structures
    summary = {
        'time_period': {'start': None, 'end': None},
        'system_status': {},
        'hardware_components': {},
        'tension_control': {
            'adjustments': defaultdict(int),
            'safety_warnings': defaultdict(int),
            'motor_positions': defaultdict(list),
            'load_cell_readings': defaultdict(list),
            'position_accuracy': defaultdict(list),  # New: track position errors
            'adjustment_timing': defaultdict(list),   # New: track timing between adjustments
            'errors_before_threshold': defaultdict(list),  # Errors during movement
            'errors_after_threshold': defaultdict(list)    # Accepted errors after threshold
        },
        'cartesian_errors': {
            'above_threshold': [],  # Error vector magnitudes > 1.5mm
            'below_threshold': []   # Error vector magnitudes <= 1.5mm
        },
        'desired_positions': [],  # Track desired position changes over time
        'position_tension_data': [],  # Track position and tension correlation
        'cable_tracking': {
            'motor_0': {'current': [], 'desired': [], 'timestamps': [], 'positions': [], 'position_timestamps': []},
            'motor_1': {'current': [], 'desired': [], 'timestamps': [], 'positions': [], 'position_timestamps': []},
            'motor_2': {'current': [], 'desired': [], 'timestamps': [], 'positions': [], 'position_timestamps': []}
        },
        'in_plane_distance': [],  # Track in-plane distance over time
        'errors_warnings': defaultdict(int),
        'performance_metrics': defaultdict(int)
    }

    # Regular expressions for parsing
    timestamp_pattern = r'\[(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\]'
    log_level_pattern = r'\[(I|D|W|E)\]:\t'

    # Motor patterns - match the actual log format
    motor_adjusted_pattern = r'Motor (\d) adjusted: current position = ([-]?\d+), new destination = ([-]?\d+)'
    motor_tension_pattern = r'Motor (\d) (underload|overload)'
    safety_warning_pattern = r'Safety: Load cell (\d) (underload|overload)'
    daq_pattern = r'DAQ - Samples/Ch: \d+ - Avg\[Ch0\]:\s*([-\d.]+), Avg\[Ch1\]:\s*([-\d.]+), Avg\[Ch2\]:\s*([-\d.]+)'
    # desired_position_pattern = r'(?:New desired position set|desired position set to centroid):\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)'
    desired_position_pattern = r'Desired position:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)'
    current_position_pattern = r'Current position:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)'
    cable_pattern = r'Base (\d)\. Current cable length/step count:\s*([-\d.]+)/\d+'
    desired_cable_pattern = r'Desired cable length/step count:\s*([-\d.]+)/\d+'
    in_plane_pattern = r'In-plane distance \(projected desired to projected current\):\s*([-\d.]+)'

    # State detection patterns - based on actual log messages
    not_close_pattern = r'Position not close enough to desired'
    error_threshold_pattern = r'Error vector magnitude ([\d.]+) > ([\d.]+) Error Threshold'
    no_adjustment_pattern = r'No tension adjustment needed'
    status_state_pattern = r'- (CALIB|RUN|MOVING|READY)'
    state_transition_pattern = r'(\w+)\s*->\s*(\w+)'  # Matches "STATE1 -> STATE2"

    # Track previous adjustment data for timing analysis
    prev_adjustment = {
        'time': None,
        'motor_positions': {},
        'motor_destinations': {}
    }

    # Track state indicators - look back context
    recent_not_close_warnings = {}  # motor_num -> timestamp of last "not close" warning
    recent_stable_indicators = {}   # motor_num -> timestamp of last "stable" indicator
    current_system_state = 'UNKNOWN'  # Initialize with UNKNOWN
    error_threshold_exceeded = False
    system_is_stable = False

    # Track last error vector magnitude to categorize it
    last_error_magnitude = None
    last_error_timestamp = None
    last_error_state = 'UNKNOWN'

    # Track recent position and DAQ data for correlation
    last_position = None
    last_desired_position = None
    last_daq_readings = None

    # Track cable length data (comes in sequence: base0 current, desired, base1 current, desired, base2 current, desired, in-plane)
    current_cable_data = {'motor': None, 'current': None, 'desired': None, 'timestamp': None}
    pending_cable_updates = []

    # Parse each line
    for line in lines:
        if not line.strip():
            continue

        # Extract timestamp
        timestamp_match = re.search(timestamp_pattern, line)
        current_time = None
        if timestamp_match:
            current_time = timestamp_match.group(1)
            if not summary['time_period']['start']:
                summary['time_period']['start'] = current_time
            summary['time_period']['end'] = current_time

        # Extract log level and message
        level_match = re.search(log_level_pattern, line)
        if level_match:
            level = level_match.group(1)
            message = line[level_match.end():]

            # Count log levels
            summary['performance_metrics'][f'log_level_{level}'] += 1

            # Detect state transitions (e.g., "MOVE -> TENSE")
            transition_match = re.search(state_transition_pattern, message)
            if transition_match:
                # Update to the new state (second state in transition)
                current_system_state = transition_match.group(2)

            # Detect system state from status updates
            state_match = re.search(status_state_pattern, message)
            if state_match:
                current_system_state = state_match.group(1)

            # First, if we have a pending error magnitude and we're moving to a new one,
            # categorize the pending one as below threshold (since no warning followed)
            if "Error vector magnitude:" in message and ">" not in message:
                # If there's a previous magnitude that wasn't categorized, it must be below threshold
                if last_error_magnitude is not None and last_error_magnitude <= 1.5:
                    summary['cartesian_errors']['below_threshold'].append({
                        'timestamp': last_error_timestamp,
                        'magnitude': last_error_magnitude,
                        'threshold': 1.5,
                        'state': last_error_state
                    })

                # Extract the new magnitude value (this is the actual Cartesian error in mm)
                mag_match = re.search(r'Error vector magnitude:\s*([\d.]+)', message)
                if mag_match:
                    mag_value = float(mag_match.group(1))
                    last_error_magnitude = mag_value
                    last_error_timestamp = current_time
                    last_error_state = current_system_state  # Store current state with error
                    # If magnitude is small (<= 1.5), consider position acceptable
                    if mag_value <= 1.5:
                        error_threshold_exceeded = False
                        system_is_stable = True

            # Detect error threshold exceeded warnings (position too far from target)
            # This warning line comes AFTER the "Error vector magnitude: X" line
            error_threshold_match = re.search(error_threshold_pattern, message)
            if error_threshold_match:
                error_magnitude = float(error_threshold_match.group(1))
                threshold = float(error_threshold_match.group(2))
                if error_magnitude > threshold:
                    error_threshold_exceeded = True
                    system_is_stable = False
                    # Categorize the last error magnitude as above threshold
                    if last_error_magnitude is not None:
                        error_data = {
                            'timestamp': last_error_timestamp,
                            'magnitude': last_error_magnitude,
                            'threshold': threshold,
                            'state': last_error_state
                        }
                        # Add current position if available
                        if last_position:
                            error_data['current_x'] = last_position['x']
                            error_data['current_y'] = last_position['y']
                            error_data['current_z'] = last_position['z']
                        # Add desired position if available
                        if last_desired_position:
                            error_data['desired_x'] = last_desired_position['x']
                            error_data['desired_y'] = last_desired_position['y']
                            error_data['desired_z'] = last_desired_position['z']
                        summary['cartesian_errors']['above_threshold'].append(error_data)
                        last_error_magnitude = None
                else:
                    # Below threshold
                    if last_error_magnitude is not None:
                        error_data = {
                            'timestamp': last_error_timestamp,
                            'magnitude': last_error_magnitude,
                            'threshold': threshold,
                            'state': last_error_state
                        }
                        # Add current position if available
                        if last_position:
                            error_data['current_x'] = last_position['x']
                            error_data['current_y'] = last_position['y']
                            error_data['current_z'] = last_position['z']
                        # Add desired position if available
                        if last_desired_position:
                            error_data['desired_x'] = last_desired_position['x']
                            error_data['desired_y'] = last_desired_position['y']
                            error_data['desired_z'] = last_desired_position['z']
                        summary['cartesian_errors']['below_threshold'].append(error_data)
                        last_error_magnitude = None

            # Detect "position not close enough" warnings
            if re.search(not_close_pattern, message):
                error_threshold_exceeded = True
                system_is_stable = False

            # Detect "no adjustment needed" (system is stable)
            if re.search(no_adjustment_pattern, message):
                system_is_stable = True
                error_threshold_exceeded = False

            # System initialization tracking
            if 'initialized successfully' in message or 'initialization complete' in message:
                component = extract_component(message)
                if component:
                    summary['hardware_components'][component] = 'OK'

            # Tension control analysis - track underload/overload events
            motor_tension_match = re.search(motor_tension_pattern, message)
            if motor_tension_match:
                motor_num = motor_tension_match.group(1)
                action = motor_tension_match.group(2)
                summary['tension_control']['adjustments'][f'motor_{motor_num}_{action}'] += 1

            # Motor adjusted messages - these contain position and destination
            motor_adjusted_match = re.search(motor_adjusted_pattern, message)
            if motor_adjusted_match and current_time:
                motor_num = motor_adjusted_match.group(1)
                current_pos = int(motor_adjusted_match.group(2))
                destination = int(motor_adjusted_match.group(3))
                position_error = abs(destination - current_pos)

                # Track all motor positions
                summary['tension_control']['motor_positions'][f'motor_{motor_num}'].append(current_pos)
                summary['tension_control']['position_accuracy'][f'motor_{motor_num}'].append(position_error)

                # Track motor positions for cable tracking graph
                motor_key = f'motor_{motor_num}'
                if motor_key in summary['cable_tracking']:
                    summary['cable_tracking'][motor_key]['positions'].append(current_pos)
                    summary['cable_tracking'][motor_key]['position_timestamps'].append(current_time)

                # Categorize error based on recent state indicators
                # If system is stable or no recent threshold warnings, this is an accepted error
                # Otherwise, it's a before-threshold error (still adjusting)
                if system_is_stable or not error_threshold_exceeded:
                    # This is an accepted error - after threshold checks passed
                    summary['tension_control']['errors_after_threshold'][f'motor_{motor_num}'].append({
                        'timestamp': current_time,
                        'error': position_error,
                        'position': current_pos,
                        'destination': destination,
                        'state': current_system_state,
                        'is_stable': system_is_stable
                    })
                else:
                    # This is a before-threshold error - still moving/adjusting
                    summary['tension_control']['errors_before_threshold'][f'motor_{motor_num}'].append({
                        'timestamp': current_time,
                        'error': position_error,
                        'position': current_pos,
                        'destination': destination,
                        'state': current_system_state,
                        'threshold_exceeded': error_threshold_exceeded
                    })

                # Store current position and destination for next timing calculation
                prev_adjustment['motor_positions'][motor_num] = current_pos
                prev_adjustment['motor_destinations'][motor_num] = destination

                # Calculate timing between adjustments for this motor
                if prev_adjustment['time'] and motor_num in prev_adjustment['motor_positions']:
                    try:
                        current_dt = datetime.strptime(current_time, '%Y-%m-%d %H:%M:%S.%f')
                        prev_dt = datetime.strptime(prev_adjustment['time'], '%Y-%m-%d %H:%M:%S.%f')
                        time_diff_ms = (current_dt - prev_dt).total_seconds() * 1000

                        # Only count if it's a reasonable time difference (not from initialization)
                        if 10 < time_diff_ms < 10000:  # Between 10ms and 10 seconds
                            summary['tension_control']['adjustment_timing'][f'motor_{motor_num}'].append(time_diff_ms)
                    except ValueError:
                        pass  # Handle timestamp parsing errors

                # Update previous adjustment time
                prev_adjustment['time'] = current_time

            # Safety warnings
            safety_match = re.search(safety_warning_pattern, message)
            if safety_match:
                cell_num = safety_match.group(1)
                warning_type = safety_match.group(2)
                summary['tension_control']['safety_warnings'][f'load_cell_{cell_num}_{warning_type}'] += 1
                summary['errors_warnings'][f'safety_{warning_type}'] += 1

            # DAQ readings
            daq_match = re.search(daq_pattern, message)
            if daq_match and current_time:
                # Store in load cell readings
                readings = [float(r) for r in daq_match.groups()]
                for i, reading in enumerate(readings):
                    summary['tension_control']['load_cell_readings'][f'ch{i}'].append({
                        'timestamp': current_time,
                        'voltage': reading
                    })

                # Keep last DAQ readings for position correlation
                last_daq_readings = readings

            # Desired position changes
            desired_pos_match = re.search(desired_position_pattern, message)
            if desired_pos_match and current_time:
                x = float(desired_pos_match.group(1))
                y = float(desired_pos_match.group(2))
                z = float(desired_pos_match.group(3))
                last_desired_position = {
                    'timestamp': current_time,
                    'x': x,
                    'y': y,
                    'z': z
                }
                summary['desired_positions'].append(last_desired_position)

            # Current position tracking
            current_pos_match = re.search(current_position_pattern, message)
            if current_pos_match and current_time:
                x = float(current_pos_match.group(1))
                y = float(current_pos_match.group(2))
                z = float(current_pos_match.group(3))
                last_position = {
                    'timestamp': current_time,
                    'x': x,
                    'y': y,
                    'z': z
                }

                # If we have recent DAQ readings, correlate them
                if last_daq_readings:
                    summary['position_tension_data'].append({
                        'timestamp': current_time,
                        'x': x,
                        'y': y,
                        'z': z,
                        'ch0': last_daq_readings[0],
                        'ch1': last_daq_readings[1],
                        'ch2': last_daq_readings[2]
                    })

            # Cable length tracking
            cable_match = re.search(cable_pattern, message)
            if cable_match and current_time:
                motor_num = int(cable_match.group(1))
                current_length = float(cable_match.group(2))
                current_cable_data = {
                    'motor': motor_num,
                    'current': current_length,
                    'desired': None,
                    'timestamp': current_time
                }

            # Desired cable length (follows current cable length)
            desired_cable_match = re.search(desired_cable_pattern, message)
            if desired_cable_match and current_cable_data['motor'] is not None:
                desired_length = float(desired_cable_match.group(1))
                current_cable_data['desired'] = desired_length

                # Store cable data
                motor_key = f"motor_{current_cable_data['motor']}"
                if motor_key in summary['cable_tracking']:
                    summary['cable_tracking'][motor_key]['current'].append(current_cable_data['current'])
                    summary['cable_tracking'][motor_key]['desired'].append(current_cable_data['desired'])
                    summary['cable_tracking'][motor_key]['timestamps'].append(current_cable_data['timestamp'])

                # Reset for next motor
                current_cable_data = {'motor': None, 'current': None, 'desired': None, 'timestamp': None}

            # In-plane distance tracking
            in_plane_match = re.search(in_plane_pattern, message)
            if in_plane_match and current_time:
                distance = float(in_plane_match.group(1))
                summary['in_plane_distance'].append({
                    'timestamp': current_time,
                    'distance': distance
                })

            # Error and warning counting
            if level in ['W', 'E']:
                error_type = classify_error(message)
                summary['errors_warnings'][error_type] += 1

    # Store log file path in summary for CSV generation
    summary['log_file_path'] = log_file_path if log_file_path else 'unknown'

    return generate_report(summary), summary

def extract_component(message):
    """Extract hardware component from initialization message"""
    components = {
        'motor controllers': 'motor' in message.lower(),
        'tracking system': 'track' in message.lower(),
        'daq system': 'daq' in message.lower(),
        'shared memory': 'shared memory' in message.lower(),
        'system': 'system' in message.lower() and 'initial' in message.lower()
    }

    for comp, condition in components.items():
        if condition:
            return comp
    return None

def classify_error(message):
    """Classify error/warning messages"""
    message_lower = message.lower()

    if 'underload' in message_lower:
        return 'tension_underload'
    elif 'overload' in message_lower:
        return 'tension_overload'
    elif 'safety' in message_lower:
        return 'safety_mechanism'
    elif 'failed' in message_lower or 'error' in message_lower:
        return 'system_error'
    else:
        return 'other_warning'

def generate_report(summary):
    """Generate a comprehensive summary report"""
    report = []

    report.append("=" * 60)
    report.append("HEART PRINTER SYSTEM LOG ANALYSIS SUMMARY")
    report.append("=" * 60)

    # Time period
    if summary['time_period']['start'] and summary['time_period']['end']:
        report.append(f"\nTIME PERIOD: {summary['time_period']['start']} to {summary['time_period']['end']}")

    # System Overview
    report.append("\n" + "=" * 40)
    report.append("SYSTEM OVERVIEW")
    report.append("=" * 40)

    total_logs = sum(summary['performance_metrics'].values())
    report.append(f"Total log entries: {total_logs}")
    report.append(f"Information entries: {summary['performance_metrics']['log_level_I']}")
    report.append(f"Warning entries: {summary['performance_metrics']['log_level_W']}")
    report.append(f"Debug entries: {summary['performance_metrics']['log_level_D']}")


    # Tension Control Analysis
    report.append("\n" + "=" * 40)
    report.append("TENSION CONTROL ANALYSIS")
    report.append("=" * 40)

    total_adjustments = sum(summary['tension_control']['adjustments'].values())
    report.append(f"Total tension adjustments: {total_adjustments}")

    report.append("\nMotor Adjustments:")
    for motor_action, count in sorted(summary['tension_control']['adjustments'].items()):
        report.append(f"  - {motor_action}: {count} times")

    report.append("\nSafety Warnings:")
    for warning, count in sorted(summary['tension_control']['safety_warnings'].items()):
        report.append(f"  - {warning}: {count} times")


    # NEW: Cartesian Error Vector Magnitude Analysis
    report.append("\n\n" + "=" * 40)
    report.append("CARTESIAN POSITION ERROR ANALYSIS")
    report.append("=" * 40)

    above_threshold = summary['cartesian_errors']['above_threshold']
    below_threshold = summary['cartesian_errors']['below_threshold']

    report.append(f"\nTotal position checks: {len(above_threshold) + len(below_threshold)}")

    # Errors ABOVE threshold (>1.5mm)
    report.append("\n ERRORS ABOVE THRESHOLD (> 1.5mm):")
    report.append("=" * 40)
    if above_threshold:
        magnitudes = [e['magnitude'] for e in above_threshold]
        avg_mag = sum(magnitudes) / len(magnitudes)
        max_mag = max(magnitudes)
        min_mag = min(magnitudes)

        report.append(f"  - Total samples: {len(magnitudes)}")
        report.append(f"  - Average error: {avg_mag:.3f} mm")
        report.append(f"  - Max error: {max_mag:.3f} mm")
        report.append(f"  - Min error: {min_mag:.3f} mm")

        # Distribution
        bins = {
            '1.5-2.0 mm': sum(1 for m in magnitudes if 1.5 < m <= 2.0),
            '2.0-5.0 mm': sum(1 for m in magnitudes if 2.0 < m <= 5.0),
            '5.0-10.0 mm': sum(1 for m in magnitudes if 5.0 < m <= 10.0),
            '>10.0 mm': sum(1 for m in magnitudes if m > 10.0)
        }
        report.append(f"  - Distribution:")
        for bin_name, count in bins.items():
            if count > 0:
                percentage = (count / len(magnitudes)) * 100
                report.append(f"    {bin_name}: {count} ({percentage:.1f}%)")
    else:
        report.append("  - No errors above threshold detected!")

    # Errors BELOW threshold (<=1.5mm)
    report.append("\n\n ERRORS BELOW THRESHOLD (<= 1.5mm) [ACCEPTED]:")
    report.append("=" * 40)
    if below_threshold:
        magnitudes = [e['magnitude'] for e in below_threshold]
        avg_mag = sum(magnitudes) / len(magnitudes)
        max_mag = max(magnitudes)
        min_mag = min(magnitudes)

        report.append(f"  - Total samples: {len(magnitudes)}")
        report.append(f"  - Average error: {avg_mag:.3f} mm")
        report.append(f"  - Max error: {max_mag:.3f} mm")
        report.append(f"  - Min error: {min_mag:.3f} mm")

        # Distribution
        bins = {
            '0.0-0.5 mm': sum(1 for m in magnitudes if m <= 0.5),
            '0.5-1.0 mm': sum(1 for m in magnitudes if 0.5 < m <= 1.0),
            '1.0-1.5 mm': sum(1 for m in magnitudes if 1.0 < m <= 1.5)
        }
        report.append(f"  - Distribution:")
        for bin_name, count in bins.items():
            if count > 0:
                percentage = (count / len(magnitudes)) * 100
                report.append(f"    {bin_name}: {count} ({percentage:.1f}%)")
    else:
        report.append("  - No errors below threshold detected!")

    # Summary statistics
    total_errors = len(above_threshold) + len(below_threshold)
    if total_errors > 0:
        acceptance_rate = (len(below_threshold) / total_errors) * 100
        report.append(f"\nSUMMARY:")
        report.append(f"  - Position acceptance rate: {acceptance_rate:.1f}% ({len(below_threshold)}/{total_errors})")
        report.append(f"  - Position rejection rate: {100-acceptance_rate:.1f}% ({len(above_threshold)}/{total_errors})")

    # NEW: Adjustment Timing Analysis
    report.append("\n" + "=" * 40)
    report.append("ADJUSTMENT TIMING ANALYSIS")
    report.append("=" * 40)

    all_timings = []
    for motor, timings in summary['tension_control']['adjustment_timing'].items():
        if timings:
            avg_time = sum(timings) / len(timings)
            max_time = max(timings)
            min_time = min(timings)
            all_timings.extend(timings)

            report.append(f"\n{motor.upper()} Adjustment Timing:")
            report.append(f"  - Average: {avg_time:.1f} ms")
            report.append(f"  - Range: {min_time:.1f} - {max_time:.1f} ms")
            report.append(f"  - Samples: {len(timings)} adjustments")

            # Categorize response time
            if avg_time <= 50:
                report.append(f"  - RESPONSE: FAST (< 50ms)")
            elif avg_time <= 200:
                report.append(f"  - RESPONSE: NORMAL (< 200ms)")
            else:
                report.append(f"  - RESPONSE: SLOW (> 200ms)")

    # Overall timing statistics
    if all_timings:
        overall_avg = sum(all_timings) / len(all_timings)
        report.append(f"\nOverall System Timing:")
        report.append(f"  - Average adjustment interval: {overall_avg:.1f} ms")
        report.append(f"  - Total timing samples: {len(all_timings)}")
        report.append(f"  - Estimated control frequency: {1000/overall_avg:.1f} Hz")

    # Error and Warning Summary
    report.append("\n" + "=" * 40)
    report.append("ERRORS AND WARNINGS SUMMARY")
    report.append("=" * 40)

    if summary['errors_warnings']:
        for error_type, count in sorted(summary['errors_warnings'].items(), key=lambda x: x[1], reverse=True):
            report.append(f"  - {error_type}: {count} occurrences")
    else:
        report.append("  No critical errors detected")

    # Performance Insights
    report.append("\n" + "=" * 40)
    report.append("PERFORMANCE INSIGHTS")
    report.append("=" * 40)

    # Calculate adjustment frequency
    if total_adjustments > 0:
        time_range = "unknown"  # Could be calculated from timestamps
        report.append(f"Tension adjustment frequency: High ({total_adjustments} adjustments)")

    # Load cell statistics
    report.append("\nLoad Cell Activity:")
    for channel, readings in summary['tension_control']['load_cell_readings'].items():
        if readings:
            voltages = [r['voltage'] for r in readings]
            avg_reading = sum(voltages) / len(voltages)
            report.append(f"  - {channel}: {len(readings)} readings, avg: {avg_reading:.4f}V")


    # Key Findings
    report.append("\n" + "=" * 40)
    report.append("KEY FINDINGS")
    report.append("=" * 40)

    findings = []

    # Check for continuous tension issues
    underload_count = summary['errors_warnings'].get('tension_underload', 0)
    overload_count = summary['errors_warnings'].get('tension_overload', 0)

    if underload_count > 100:
        findings.append(" SIGNIFICANT UNDERLOAD ISSUES: System frequently detects insufficient tension")
    if overload_count > 50:
        findings.append(" FREQUENT OVERLOAD CONDITIONS: Motors experiencing excessive tension")

    if total_adjustments > 500:
        findings.append(" HIGH ADJUSTMENT FREQUENCY: System constantly correcting tension levels")

    # NEW: Cartesian position accuracy findings
    above_threshold = summary['cartesian_errors']['above_threshold']
    below_threshold = summary['cartesian_errors']['below_threshold']
    total_checks = len(above_threshold) + len(below_threshold)

    if total_checks > 0:
        acceptance_rate = (len(below_threshold) / total_checks) * 100

        if below_threshold:
            avg_accepted_error = sum(e['magnitude'] for e in below_threshold) / len(below_threshold)
            if avg_accepted_error < 0.5:
                findings.append(f" EXCELLENT CARTESIAN ACCURACY: Average accepted error of {avg_accepted_error:.3f}mm")
            elif avg_accepted_error < 1.0:
                findings.append(f" GOOD CARTESIAN ACCURACY: Average accepted error of {avg_accepted_error:.3f}mm")
            else:
                findings.append(f" MODERATE CARTESIAN ACCURACY: Average accepted error of {avg_accepted_error:.3f}mm")

        if acceptance_rate >= 90:
            findings.append(f" HIGH POSITION ACCEPTANCE RATE: {acceptance_rate:.1f}% of positions accepted")
        elif acceptance_rate >= 70:
            findings.append(f" MODERATE POSITION ACCEPTANCE RATE: {acceptance_rate:.1f}% of positions accepted")
        else:
            findings.append(f" LOW POSITION ACCEPTANCE RATE: Only {acceptance_rate:.1f}% of positions accepted")

        if above_threshold:
            avg_rejected_error = sum(e['magnitude'] for e in above_threshold) / len(above_threshold)
            if avg_rejected_error > 10:
                findings.append(f" LARGE POSITION ERRORS DETECTED: Average rejected error of {avg_rejected_error:.1f}mm")

    # NEW: Timing findings
    if all_timings:
        overall_avg = sum(all_timings) / len(all_timings)
        if overall_avg > 500:
            findings.append(" SLOW RESPONSE: Adjustment intervals exceeding 500ms")
        elif overall_avg <= 50:
            findings.append(" FAST RESPONSE: System responding quickly to tension changes")

    # Check if system is generally stable despite adjustments
    if underload_count + overload_count < total_adjustments * 0.1:
        findings.append(" SYSTEM STABILITY: Tension control is actively maintaining stable operation")
    else:
        findings.append(" TENSION INSTABILITY: Frequent safety threshold violations")

    if not findings:
        findings.append(" SYSTEM OPERATING NORMALLY: No major issues detected")

    for finding in findings:
        report.append(finding)

    return "\n".join(report)

def generate_csv_from_summaries(summaries, output_csv_path):
    """
    Generate a CSV file with all positional errors from multiple log analyses
    """
    csv_rows = []

    for summary in summaries:
        log_file = summary.get('log_file_path', 'unknown')

        # Process above threshold errors (rejected)
        for error_data in summary['cartesian_errors']['above_threshold']:
            timestamp_str = error_data['timestamp']
            try:
                dt = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
                unix_time = int(dt.timestamp())
            except:
                unix_time = 0

            csv_rows.append({
                'log_file': os.path.basename(log_file),
                'unix_timestamp': unix_time,
                'datetime': timestamp_str,
                'magnitude_mm': error_data['magnitude'],
                'threshold_mm': error_data.get('threshold', 1.5),
                'status': 'rejected',
                'state': error_data.get('state', 'UNKNOWN'),
                'current_x': error_data.get('current_x', ''),
                'current_y': error_data.get('current_y', ''),
                'current_z': error_data.get('current_z', ''),
                'desired_x': error_data.get('desired_x', ''),
                'desired_y': error_data.get('desired_y', ''),
                'desired_z': error_data.get('desired_z', '')
            })

        # Process below threshold errors (accepted)
        for error_data in summary['cartesian_errors']['below_threshold']:
            timestamp_str = error_data['timestamp']
            try:
                dt = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S.%f')
                unix_time = int(dt.timestamp())
            except:
                unix_time = 0

            csv_rows.append({
                'log_file': os.path.basename(log_file),
                'unix_timestamp': unix_time,
                'datetime': timestamp_str,
                'magnitude_mm': error_data['magnitude'],
                'threshold_mm': error_data.get('threshold', 1.5),
                'status': 'accepted',
                'state': error_data.get('state', 'UNKNOWN'),
                'current_x': error_data.get('current_x', ''),
                'current_y': error_data.get('current_y', ''),
                'current_z': error_data.get('current_z', ''),
                'desired_x': error_data.get('desired_x', ''),
                'desired_y': error_data.get('desired_y', ''),
                'desired_z': error_data.get('desired_z', '')
            })

    # Sort by unix timestamp
    csv_rows.sort(key=lambda x: x['unix_timestamp'])

    # Write CSV
    with open(output_csv_path, 'w', newline='') as csvfile:
        fieldnames = ['log_file', 'unix_timestamp', 'datetime', 'magnitude_mm', 'threshold_mm', 'status', 'state',
                      'current_x', 'current_y', 'current_z', 'desired_x', 'desired_y', 'desired_z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for row in csv_rows:
            writer.writerow(row)

    print(f"CSV file generated: {output_csv_path}")
    print(f"Total positional errors recorded: {len(csv_rows)}")

def generate_tension_graphs(summary, output_path):
    """
    Generate graphs showing tension (load cell voltage) over time for each channel
    """
    load_cell_readings = summary['tension_control']['load_cell_readings']
    desired_positions = summary.get('desired_positions', [])

    # Check if we have any load cell data
    if not any(load_cell_readings.values()):
        print("No load cell data available for graphing")
        return

    # Create figure with subplots for each channel
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle('Load Cell Tension Over Time with Desired Position Changes', fontsize=16, fontweight='bold')

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green

    for ch_idx, (channel, readings) in enumerate(sorted(load_cell_readings.items())):
        if not readings:
            continue

        # Extract timestamps and voltages
        timestamps = []
        voltages = []

        for reading in readings:
            try:
                dt = datetime.strptime(reading['timestamp'], '%Y-%m-%d %H:%M:%S.%f')
                timestamps.append(dt)
                voltages.append(reading['voltage'])
            except (ValueError, KeyError) as e:
                continue

        if not timestamps:
            continue

        # Plot on corresponding subplot
        ax = axes[ch_idx]
        ax.plot(timestamps, voltages, color=colors[ch_idx], linewidth=0.8, alpha=0.7, label='Tension')
        ax.set_ylabel(f'Channel {ch_idx} (V)', fontweight='bold')
        ax.grid(True, alpha=0.3)

        # Add desired position change markers
        if desired_positions:
            for i, pos_change in enumerate(desired_positions):
                try:
                    pos_time = datetime.strptime(pos_change['timestamp'], '%Y-%m-%d %H:%M:%S.%f')

                    # Draw vertical line at position change time
                    ax.axvline(x=pos_time, color='red', linestyle='--', linewidth=1.5, alpha=0.6)

                    # Add text annotation for first few position changes (avoid clutter)
                    if i < 5:  # Only annotate first 5 changes
                        pos_text = f"({pos_change['x']:.1f}, {pos_change['y']:.1f}, {pos_change['z']:.1f})"
                        ax.text(pos_time, ax.get_ylim()[1] * 0.85, pos_text,
                               rotation=90, fontsize=7, verticalalignment='bottom',
                               color='red', alpha=0.8)
                except (ValueError, KeyError):
                    continue

        # Add statistics to plot
        avg_voltage = sum(voltages) / len(voltages)
        max_voltage = max(voltages)
        min_voltage = min(voltages)

        stats_text = f'Avg: {avg_voltage:.3f}V | Min: {min_voltage:.3f}V | Max: {max_voltage:.3f}V'
        ax.text(0.02, 0.95, stats_text, transform=ax.transAxes,
                fontsize=9, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        # Add legend only on first subplot
        if ch_idx == 0 and desired_positions:
            from matplotlib.lines import Line2D
            legend_elements = [
                Line2D([0], [0], color=colors[ch_idx], linewidth=2, label='Tension'),
                Line2D([0], [0], color='red', linestyle='--', linewidth=1.5, label='Desired Position Change')
            ]
            ax.legend(handles=legend_elements, loc='upper right', fontsize=8)

        # Format y-axis
        ax.set_ylim(min_voltage - 0.5, max_voltage + 0.5)

    # Format x-axis (time)
    axes[-1].set_xlabel('Time', fontweight='bold')
    axes[-1].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    plt.setp(axes[-1].xaxis.get_majorticklabels(), rotation=45, ha='right')

    # Adjust layout and save
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"Tension graph saved to: {output_path}")
    if desired_positions:
        print(f"  - Marked {len(desired_positions)} desired position change(s)")

def generate_cable_tracking_graph(summary, output_path):
    """
    Generate graphs showing current/desired cable lengths and in-plane distance over time
    """
    cable_data = summary.get('cable_tracking', {})
    in_plane_data = summary.get('in_plane_distance', [])

    # Check if we have cable tracking data
    has_data = any(cable_data.get(f'motor_{i}', {}).get('current', []) for i in range(3))

    if not has_data and not in_plane_data:
        print("No cable tracking data available for graphing")
        return

    # Create figure with 4 subplots (3 motors + 1 in-plane distance)
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle('Cable Length Tracking and In-Plane Distance', fontsize=16, fontweight='bold')

    colors_current = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green
    colors_desired = ['#4a90e2', '#ff9f50', '#5fbf5f']  # Lighter versions

    # Plot cable lengths for each motor
    for motor_idx in range(3):
        motor_key = f'motor_{motor_idx}'
        if motor_key not in cable_data:
            continue

        timestamps_str = cable_data[motor_key].get('timestamps', [])
        current_lengths = cable_data[motor_key].get('current', [])
        desired_lengths = cable_data[motor_key].get('desired', [])

        if not timestamps_str:
            continue

        # Convert timestamps
        timestamps = []
        for ts_str in timestamps_str:
            try:
                dt = datetime.strptime(ts_str, '%Y-%m-%d %H:%M:%S.%f')
                timestamps.append(dt)
            except ValueError:
                continue

        if not timestamps:
            continue

        ax = axes[motor_idx]

        # Plot current and desired cable lengths
        ax.plot(timestamps, current_lengths, color=colors_current[motor_idx],
               linewidth=1.5, label='Current Length', marker='o', markersize=2, alpha=0.7)
        ax.plot(timestamps, desired_lengths, color=colors_desired[motor_idx],
               linewidth=1.5, label='Desired Length', linestyle='--', marker='s', markersize=2, alpha=0.7)

        ax.set_ylabel(f'Motor {motor_idx}\nLength (mm)', fontweight='bold')
        ax.grid(True, alpha=0.3)

        # Add motor positions on secondary y-axis
        position_timestamps_str = cable_data[motor_key].get('position_timestamps', [])
        motor_positions = cable_data[motor_key].get('positions', [])

        if position_timestamps_str and motor_positions:
            # Convert position timestamps
            position_timestamps = []
            valid_positions = []
            for ts_str, pos in zip(position_timestamps_str, motor_positions):
                try:
                    dt = datetime.strptime(ts_str, '%Y-%m-%d %H:%M:%S.%f')
                    position_timestamps.append(dt)
                    valid_positions.append(pos)
                except ValueError:
                    continue

            if position_timestamps:
                # Create secondary y-axis for motor positions
                ax2 = ax.twinx()
                ax2.plot(position_timestamps, valid_positions, color='purple',
                        linewidth=1.0, label='Motor Position', marker='^', markersize=2,
                        alpha=0.5, linestyle='-.')
                ax2.set_ylabel('Position (steps)', fontweight='bold', color='purple')
                ax2.tick_params(axis='y', labelcolor='purple')

                # Combine legends from both axes
                lines1, labels1 = ax.get_legend_handles_labels()
                lines2, labels2 = ax2.get_legend_handles_labels()
                ax.legend(lines1 + lines2, labels1 + labels2, loc='upper right', fontsize=8)
        else:
            ax.legend(loc='upper right', fontsize=8)

        # Add statistics
        if current_lengths and desired_lengths:
            avg_current = sum(current_lengths) / len(current_lengths)
            avg_desired = sum(desired_lengths) / len(desired_lengths)
            avg_error = sum(abs(c - d) for c, d in zip(current_lengths, desired_lengths)) / len(current_lengths)

            stats_text = f'Avg Current: {avg_current:.1f}mm\nAvg Desired: {avg_desired:.1f}mm\nAvg Error: {avg_error:.2f}mm'
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
                   fontsize=8, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

    # Plot in-plane distance on 4th subplot
    if in_plane_data:
        ax = axes[3]

        timestamps = []
        distances = []

        for data_point in in_plane_data:
            try:
                dt = datetime.strptime(data_point['timestamp'], '%Y-%m-%d %H:%M:%S.%f')
                timestamps.append(dt)
                distances.append(data_point['distance'])
            except (ValueError, KeyError):
                continue

        if timestamps:
            ax.plot(timestamps, distances, color='#d62728', linewidth=1.5,
                   marker='o', markersize=2, alpha=0.7)
            ax.set_ylabel('In-Plane\nDistance (mm)', fontweight='bold')
            ax.grid(True, alpha=0.3)

            # Add statistics
            avg_dist = sum(distances) / len(distances)
            max_dist = max(distances)
            min_dist = min(distances)

            stats_text = f'Avg: {avg_dist:.2f}mm\nMin: {min_dist:.2f}mm\nMax: {max_dist:.2f}mm'
            ax.text(0.02, 0.98, stats_text, transform=ax.transAxes,
                   fontsize=8, verticalalignment='top',
                   bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7))

    # Format x-axis (time)
    axes[-1].set_xlabel('Time', fontweight='bold')
    axes[-1].xaxis.set_major_formatter(mdates.DateFormatter('%H:%M:%S'))
    plt.setp(axes[-1].xaxis.get_majorticklabels(), rotation=45, ha='right')

    # Adjust layout and save
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    plt.close()

    print(f"Cable tracking graph saved to: {output_path}")
    if in_plane_data:
        print(f"  - {len(in_plane_data)} in-plane distance measurements")

# Example usage with your log file:
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Analyze Heart Printer log files')
    parser.add_argument('log_path', help='Path or glob pattern to log file(s) (e.g., "logs/*.txt")')
    parser.add_argument('-o', '--output-dir', default=None,
                        help='Output directory for analysis files (default: ./analysis)')

    args = parser.parse_args()

    # Determine output directory
    if args.output_dir:
        output_dir = args.output_dir
    else:
        output_dir = os.path.join(os.getcwd(), 'analysis')

    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    print(f"Output directory: {output_dir}")

    # Find all matching log files
    log_files = glob.glob(args.log_path)

    if not log_files:
        print(f"No log files found matching pattern: {args.log_path}")
        sys.exit(1)

    print(f"Found {len(log_files)} file(s) to process")

    # Process each log file
    all_summaries = []
    processed_count = 0

    for log_file in log_files:
        print(f"\nProcessing: {log_file}")

        # Validate log file
        if not is_valid_log_file(log_file):
            print(f"Skipping - not a valid Heart Printer log file")
            continue

        try:
            # Read log file
            with open(log_file, 'r') as f:
                log_content = f.read()

            # Analyze log
            report, summary = analyze_heart_printer_logs(log_content, log_file)
            all_summaries.append(summary)

            # Generate output filename
            log_basename = os.path.splitext(os.path.basename(log_file))[0]
            output_file = os.path.join(output_dir, f"{log_basename}_analysis.txt")

            # Write analysis report
            with open(output_file, 'w') as f:
                f.write(report)

            print(f"Analysis saved to: {output_file}")

            # Generate tension graphs
            graph_output_file = os.path.join(output_dir, f"{log_basename}_tension.png")
            generate_tension_graphs(summary, graph_output_file)

            # Generate cable tracking graph
            cable_graph_file = os.path.join(output_dir, f"{log_basename}_cable_tracking.png")
            generate_cable_tracking_graph(summary, cable_graph_file)

            processed_count += 1

        except Exception as e:
            print(f"Error processing file: {e}")
            continue

    # Generate combined CSV
    if all_summaries:
        csv_output_path = os.path.join(output_dir, 'positional_errors_combined.csv')
        generate_csv_from_summaries(all_summaries, csv_output_path)

    print(f"\n{'='*60}")
    print(f"Processing complete!")
    print(f"Files processed: {processed_count}/{len(log_files)}")
    print(f"Output directory: {output_dir}")
    print(f"{'='*60}")