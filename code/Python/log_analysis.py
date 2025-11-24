import re
from datetime import datetime
from collections import defaultdict, Counter
import sys
import glob
import os
import csv

def analyze_heart_printer_logs(log_content):
    """
    Analyze Heart Printer System logs and provide a comprehensive summary
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

    # State detection patterns - based on actual log messages
    not_close_pattern = r'Position not close enough to desired'
    error_threshold_pattern = r'Error vector magnitude ([\d.]+) > ([\d.]+) Error Threshold'
    no_adjustment_pattern = r'No tension adjustment needed'
    status_state_pattern = r'- (CALIB|RUN|MOVING|READY)'

    # Track previous adjustment data for timing analysis
    prev_adjustment = {
        'time': None,
        'motor_positions': {},
        'motor_destinations': {}
    }

    # Track state indicators - look back context
    recent_not_close_warnings = {}  # motor_num -> timestamp of last "not close" warning
    recent_stable_indicators = {}   # motor_num -> timestamp of last "stable" indicator
    current_system_state = None
    error_threshold_exceeded = False
    system_is_stable = False

    # Track last error vector magnitude to categorize it
    last_error_magnitude = None
    last_error_timestamp = None

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
                        'threshold': 1.5
                    })

                # Extract the new magnitude value (this is the actual Cartesian error in mm)
                mag_match = re.search(r'Error vector magnitude:\s*([\d.]+)', message)
                if mag_match:
                    mag_value = float(mag_match.group(1))
                    last_error_magnitude = mag_value
                    last_error_timestamp = current_time
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
                        summary['cartesian_errors']['above_threshold'].append({
                            'timestamp': last_error_timestamp,
                            'magnitude': last_error_magnitude,
                            'threshold': threshold
                        })
                        last_error_magnitude = None
                else:
                    # Below threshold
                    if last_error_magnitude is not None:
                        summary['cartesian_errors']['below_threshold'].append({
                            'timestamp': last_error_timestamp,
                            'magnitude': last_error_magnitude,
                            'threshold': threshold
                        })
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
            if daq_match:
                for i, reading in enumerate(daq_match.groups()):
                    summary['tension_control']['load_cell_readings'][f'ch{i}'].append(float(reading))

            # Error and warning counting
            if level in ['W', 'E']:
                error_type = classify_error(message)
                summary['errors_warnings'][error_type] += 1

    return generate_report(summary)

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
    report.append("\n🔴 ERRORS ABOVE THRESHOLD (> 1.5mm):")
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
                report.append(f"    • {bin_name}: {count} ({percentage:.1f}%)")
    else:
        report.append("  - No errors above threshold detected!")

    # Errors BELOW threshold (<=1.5mm)
    report.append("\n\n✅ ERRORS BELOW THRESHOLD (<= 1.5mm) [ACCEPTED]:")
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
                report.append(f"    • {bin_name}: {count} ({percentage:.1f}%)")
    else:
        report.append("  - No errors below threshold detected!")

    # Summary statistics
    total_errors = len(above_threshold) + len(below_threshold)
    if total_errors > 0:
        acceptance_rate = (len(below_threshold) / total_errors) * 100
        report.append(f"\n📈 SUMMARY:")
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
                report.append(f"  - RESPONSE: ⚡ FAST (≤ 50ms)")
            elif avg_time <= 200:
                report.append(f"  - RESPONSE: 📊 NORMAL (≤ 200ms)")
            else:
                report.append(f"  - RESPONSE: ⚠️  SLOW (> 200ms)")

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
            avg_reading = sum(readings) / len(readings)
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
        findings.append("⚠️  SIGNIFICANT UNDERLOAD ISSUES: System frequently detects insufficient tension")
    if overload_count > 50:
        findings.append("⚠️  FREQUENT OVERLOAD CONDITIONS: Motors experiencing excessive tension")

    if total_adjustments > 500:
        findings.append("🔧  HIGH ADJUSTMENT FREQUENCY: System constantly correcting tension levels")

    # NEW: Cartesian position accuracy findings
    above_threshold = summary['cartesian_errors']['above_threshold']
    below_threshold = summary['cartesian_errors']['below_threshold']
    total_checks = len(above_threshold) + len(below_threshold)

    if total_checks > 0:
        acceptance_rate = (len(below_threshold) / total_checks) * 100

        if below_threshold:
            avg_accepted_error = sum(e['magnitude'] for e in below_threshold) / len(below_threshold)
            if avg_accepted_error < 0.5:
                findings.append(f"✅  EXCELLENT CARTESIAN ACCURACY: Average accepted error of {avg_accepted_error:.3f}mm")
            elif avg_accepted_error < 1.0:
                findings.append(f"✅  GOOD CARTESIAN ACCURACY: Average accepted error of {avg_accepted_error:.3f}mm")
            else:
                findings.append(f"📊  MODERATE CARTESIAN ACCURACY: Average accepted error of {avg_accepted_error:.3f}mm")

        if acceptance_rate >= 90:
            findings.append(f"✅  HIGH POSITION ACCEPTANCE RATE: {acceptance_rate:.1f}% of positions accepted")
        elif acceptance_rate >= 70:
            findings.append(f"📊  MODERATE POSITION ACCEPTANCE RATE: {acceptance_rate:.1f}% of positions accepted")
        else:
            findings.append(f"⚠️  LOW POSITION ACCEPTANCE RATE: Only {acceptance_rate:.1f}% of positions accepted")

        if above_threshold:
            avg_rejected_error = sum(e['magnitude'] for e in above_threshold) / len(above_threshold)
            if avg_rejected_error > 10:
                findings.append(f"⚠️  LARGE POSITION ERRORS DETECTED: Average rejected error of {avg_rejected_error:.1f}mm")

    # NEW: Timing findings
    if all_timings:
        overall_avg = sum(all_timings) / len(all_timings)
        if overall_avg > 500:
            findings.append("⚠️  SLOW RESPONSE: Adjustment intervals exceeding 500ms")
        elif overall_avg <= 50:
            findings.append("✅  FAST RESPONSE: System responding quickly to tension changes")

    # Check if system is generally stable despite adjustments
    if underload_count + overload_count < total_adjustments * 0.1:
        findings.append("✅  SYSTEM STABILITY: Tension control is actively maintaining stable operation")
    else:
        findings.append("⚠️  TENSION INSTABILITY: Frequent safety threshold violations")

    if not findings:
        findings.append("✅  SYSTEM OPERATING NORMALLY: No major issues detected")

    for finding in findings:
        report.append(finding)

    return "\n".join(report)

# Example usage with your log file:
if __name__ == "__main__":
    # Read the log file
    if(len(sys.argv) > 1):
        log_file = sys.argv[1]
    else:
        print("Missing log path.")
        exit(0)

    with open(log_file, 'r') as file:
        log_content = file.read()

    # Analyze and print summary
    summary_report = analyze_heart_printer_logs(log_content)
    print(summary_report)