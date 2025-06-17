import time
import random
import os

def generate_non_collinear_points():
    """
    Generates three fixed, non-collinear points in 3D space.
    For simplicity, these points form a right-angled triangle in the XY plane.
    """
    # Define three fixed, non-collinear points
    point_A = (0.0, 0.0, 0.0)
    point_B = (5.0, 0.0, 0.0)
    point_C = (0.0, 5.0, 0.0) # Z is 0 to keep it simple, but they can have different Z values.

    # Print to console for initial setup visibility
    print(f"Fixed Point A: {point_A}")
    print(f"Fixed Point B: {point_B}")
    print(f"Fixed Point C: {point_C}")
    return point_A, point_B, point_C

def generate_bounded_point_in_triangle(p_a, p_b, p_c, max_z_travel =15.0):
    """
    Generates a new 3D point (x, y, z) that lies within the triangle
    formed by points p_a, p_b, and p_c using barycentric coordinates.

    Args:
        p_a (tuple): (x, y, z) coordinates of the first point.
        p_b (tuple): (x, y, z) coordinates of the second point.
        p_c (tuple): (x, y, z) coordinates of the third point.

    Returns:
        tuple: (x, y, z) coordinates of the new point.
    """
    # Generate two random numbers between 0 and 1
    s = random.random()
    t = random.random()

    # If s + t > 1, reflect them to ensure the point is within the triangle
    # This transforms a point in the parallelogram (0,0)-(1,0)-(1,1)-(0,1)
    # into a point within the triangle (0,0)-(1,0)-(0,1)
    if s + t > 1:
        s = 1 - s
        t = 1 - t

    # Calculate barycentric coordinates
    # u, v, w are the weights for point A, B, and C respectively.
    # The new point P = u*P_a + v*P_b + w*P_c
    u = 1 - s - t # Weight for point A
    v = s         # Weight for point B
    w = t         # Weight for point C

    # Calculate the new point's coordinates
    new_x = u * p_a[0] + v * p_b[0] + w * p_c[0]
    new_y = u * p_a[1] + v * p_b[1] + w * p_c[1]
    new_z = u * p_a[2] + v * p_b[2] + w * p_c[2]
    z_offset = random.uniform(0.0, max_z_travel)
    new_z +=  z_offset
    return new_x, new_y, new_z

def generate_random_rpy():
    """
    Generates random roll, pitch, and yaw values in degrees.
    Angles are typically in the range of -180 to 180 degrees.

    Returns:
        tuple: (roll, pitch, yaw) in degrees.
    """
    roll = random.uniform(-180.0, 180.0)
    pitch = random.uniform(-180.0, 180.0)
    yaw = random.uniform(-180.0, 180.0)
    return roll, pitch, yaw

def main():
    """
    Main function to run the point generation and update loop,
    saving the output to a file.
    """
    point_A, point_B, point_C = generate_non_collinear_points()

    update_interval_ms = 50 # milliseconds
    update_interval_sec = update_interval_ms / 1000.0 # convert to seconds
    output_filename = "input.txt" # Name of the file to save the output

    print(f"\nStarting generation loop (updates every {update_interval_ms}ms).")
    print(f"Output will be saved to '{output_filename}' (overwritten each time). Press Ctrl+C to stop.")

    try:
        while True:
            # Generate the 4th point bounded by A, B, C
            new_point_xyz = generate_bounded_point_in_triangle(point_A, point_B, point_C)

            # Generate random roll, pitch, and yaw
            roll, pitch, yaw = generate_random_rpy()

            # Format the output string
            output_string = (
                f"{point_A[0]:.3f}, {point_A[1]:.3f},{point_A[2]:.3f},"
                f"{0.0:.2f},{0.0:.2f}, {0.0:.2f}\n"
                f"{point_B[0]:.3f}, {point_B[1]:.3f},{point_B[2]:.3f},"
                f"{0.0:.2f},{0.0:.2f}, {0.0:.2f}\n"
                f"{point_C[0]:.3f}, {point_C[1]:.3f},{point_C[2]:.3f},"
                f"{0.0:.2f},{0.0:.2f}, {0.0:.2f}\n"
                f"{new_point_xyz[0]:.3f}, {new_point_xyz[1]:.3f},{new_point_xyz[2]:.3f},"
                f"{roll:.2f},{pitch:.2f}, {yaw:.2f}\n"
            )

            # Save the output to the file, overwriting its content each time
            try:
                with open(output_filename, 'w') as f:
                    f.write(output_string)
                # Print to console for immediate feedback as well
                print(f"Saved: {output_string.strip()}")
            except IOError as e:
                print(f"Error writing to '{output_filename}': {e}")

            # Wait for the specified interval
            time.sleep(update_interval_sec)

    except KeyboardInterrupt:
        print("\nGeneration stopped by user.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    main()
