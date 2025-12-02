import pandas as pd
import numpy as np

# Read the CSV file
csv_file = "positional_errors_combined.csv"
df = pd.read_csv(csv_file)

# Pattern for each 14-row trial:
# Row 0: centroid
# Row 1: centroid
# Row 2: Right
# Row 3: centroid
# Row 4: Center
# Row 5: centroid
# Row 6: Left
# Row 7: centroid
# Row 8: Center-Right
# Row 9: centroid
# Row 10: Center-Left
# Row 11: centroid
# Row 12: Left-Right
# Row 13: centroid

def assign_base_label(index):
    """Assign base label based on position in 14-row trial pattern."""
    position_in_trial = index % 14

    if position_in_trial in [0, 1, 3, 5, 7, 9, 11, 13]:
        return "Centroid"
    elif position_in_trial == 2:
        return "Right"
    elif position_in_trial == 4:
        return "Center"
    elif position_in_trial == 6:
        return "Left"
    elif position_in_trial == 8:
        return "Center-Right"
    elif position_in_trial == 10:
        return "Center-Left"
    elif position_in_trial == 12:
        return "Left-Right"
    else:
        return "unknown"

def print_statistics(data, label):
    """Print statistics for error_magnitude_mm and time_to_reach_sec."""
    print(f"\n{'='*60}")
    print(f"{label}")
    print(f"{'='*60}")
    print(f"Count: {len(data)}")

    print(f"\nError Magnitude (mm):")
    print(f"  Mean:   {data['error_magnitude_mm'].mean():.4f}")
    print(f"  Median: {data['error_magnitude_mm'].median():.4f}")
    print(f"  Std:    {data['error_magnitude_mm'].std():.4f}")
    print(f"  Min:    {data['error_magnitude_mm'].min():.4f}")
    print(f"  Max:    {data['error_magnitude_mm'].max():.4f}")

    print(f"\nTime to Reach (sec):")
    print(f"  Mean:   {data['time_to_reach_sec'].mean():.4f}")
    print(f"  Median: {data['time_to_reach_sec'].median():.4f}")
    print(f"  Std:    {data['time_to_reach_sec'].std():.4f}")
    print(f"  Min:    {data['time_to_reach_sec'].min():.4f}")
    print(f"  Max:    {data['time_to_reach_sec'].max():.4f}")

    print(f"\nStarting Distance (mm):")
    print(f"  Mean:   {data['starting_distance_mm'].mean():.4f}")
    print(f"  Median: {data['starting_distance_mm'].median():.4f}")
    print(f"  Std:    {data['starting_distance_mm'].std():.4f}")
    print(f"  Min:    {data['starting_distance_mm'].min():.4f}")
    print(f"  Max:    {data['starting_distance_mm'].max():.4f}")

def calculate_statistics(data):
    """Calculate statistics and return as a dictionary."""
    return {
        'count': len(data),
        'error_mean': data['error_magnitude_mm'].mean(),
        'error_median': data['error_magnitude_mm'].median(),
        'error_std': data['error_magnitude_mm'].std(),
        'error_min': data['error_magnitude_mm'].min(),
        'error_max': data['error_magnitude_mm'].max(),
        'time_mean': data['time_to_reach_sec'].mean(),
        'time_median': data['time_to_reach_sec'].median(),
        'time_std': data['time_to_reach_sec'].std(),
        'time_min': data['time_to_reach_sec'].min(),
        'time_max': data['time_to_reach_sec'].max(),
        'distance_mean': data['starting_distance_mm'].mean(),
        'distance_median': data['starting_distance_mm'].median(),
        'distance_std': data['starting_distance_mm'].std(),
        'distance_min': data['starting_distance_mm'].min(),
        'distance_max': data['starting_distance_mm'].max()
    }

def main():
    """Main function to run the analysis."""
    # Assign base labels to each row
    df['base'] = df.index.map(assign_base_label)

    # Collect statistics for CSV output
    stats_list = []

    # 1. Overall statistics
    print_statistics(df, "OVERALL STATISTICS")
    overall_stats = calculate_statistics(df)
    overall_stats['position'] = 'overall'
    stats_list.append(overall_stats)

    # 2. Statistics for each base
    for base in ["Centroid", "Right", "Center", "Left", "Center-Right", "Center-Left", "Left-Right"]:
        base_data = df[df['base'] == base]
        if len(base_data) > 0:
            label = f"STATISTICS FOR {base.upper()}"
            print_statistics(base_data, label)
            base_stats = calculate_statistics(base_data)
            base_stats['position'] = base
            stats_list.append(base_stats)

    # Create DataFrame from statistics and save to CSV
    stats_df = pd.DataFrame(stats_list)
    # Reorder columns to have position first
    cols = ['position'] + [col for col in stats_df.columns if col != 'position']
    stats_df = stats_df[cols]

    output_file = "position_error_statistics.csv"
    stats_df.to_csv(output_file, index=False)

    print(f"\n{'='*60}")
    print(f"Analysis complete!")
    print(f"Statistics saved to: {output_file}")
    print(f"{'='*60}")

if __name__ == "__main__":
    main()
