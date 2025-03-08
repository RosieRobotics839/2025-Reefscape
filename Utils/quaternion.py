import math
import sys

def rotation_quaternion(axis, angle_degrees):
    """
    Calculate the quaternion for a rotation around a specified axis.
    
    Parameters:
    axis (str or list): The rotation axis ('x', 'y', 'z') or a normalized 3D vector [x, y, z]
    angle_degrees (float): The rotation angle in degrees
    
    Returns:
    list: The quaternion in the form [w, x, y, z]
    """
    # Convert angle to radians
    angle_radians = math.radians(angle_degrees)
    
    # Calculate the half angle
    half_angle = angle_radians / 2.0
    
    # Calculate sine and cosine of half angle
    cos_half = math.cos(half_angle)
    sin_half = math.sin(half_angle)
    
    # Process the axis
    if isinstance(axis, str):
        if axis.lower() == 'x':
            axis_vector = [1, 0, 0]
        elif axis.lower() == 'y':
            axis_vector = [0, 1, 0]
        elif axis.lower() == 'z':
            axis_vector = [0, 0, 1]
        else:
            raise ValueError("Axis must be 'x', 'y', 'z', or a 3D vector")
    else:
        axis_vector = axis
        # Normalize the axis vector
        norm = math.sqrt(sum(v*v for v in axis_vector))
        if norm == 0:
            raise ValueError("Axis vector cannot be zero")
        axis_vector = [v/norm for v in axis_vector]
    
    # Calculate the quaternion
    w = cos_half
    x = sin_half * axis_vector[0]
    y = sin_half * axis_vector[1]
    z = sin_half * axis_vector[2]
    
    return [w, x, y, z]

if __name__ == "__main__":
    # Check if command line arguments are provided
    if len(sys.argv) < 3:
        print("Usage: python script.py <axis> <angle_degrees>")
        print("Example: python script.py z 45")
        print("         python script.py 0,0,1 45")
        sys.exit(1)
    
    # Parse the first argument as axis
    axis_arg = sys.argv[1]
    if ',' in axis_arg:
        # If comma-separated, interpret as vector
        try:
            axis = [float(val) for val in axis_arg.split(',')]
            if len(axis) != 3:
                raise ValueError("Vector axis must have 3 components")
        except ValueError as e:
            print(f"Error parsing axis vector: {e}")
            sys.exit(1)
    else:
        # Otherwise, interpret as x, y, or z
        axis = axis_arg
    
    # Parse the second argument as angle
    try:
        angle = float(sys.argv[2])
    except ValueError:
        print("Error: Angle must be a number")
        sys.exit(1)
    
    # Calculate and print quaternion
    try:
        quaternion = rotation_quaternion(axis, angle)
        print(f"Quaternion for {angle}° rotation around {axis}: {quaternion}")
        print(f"[w, x, y, z] = [{quaternion[0]:.6f}, {quaternion[1]:.6f}, {quaternion[2]:.6f}, {quaternion[3]:.6f}]")
    except ValueError as e:
        print(f"Error calculating quaternion: {e}")
        sys.exit(1)