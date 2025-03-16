# main.py - The main application file

from networktables import NetworkTables
import serial
import time
import json

# Eli TODO: Set the correct robot IP and serial port
ROBOT_IP = "10.XX.XX.2"  # Replace with your team's robot IP
ARDUINO_PORT = "COM3"    # Change this to match your Arduino's port
BAUD_RATE = 9600         # Arduino baud rate

def connect_to_networktables(ip):
    """Connect to the NetworkTables server on the robot"""
    NetworkTables.initialize(server=ip)
    
    # Wait briefly for connection to establish
    time.sleep(0.5)
    
    # Return True if connected, False otherwise
    return NetworkTables.isConnected()

def get_robot_data():
    """
    Retrieve the specific data we want from NetworkTables
    
    Eli TODO: Modify this function to get the exact data you need
    """
    try:
        # Example: get some motor data
        drivetrain = NetworkTables.getTable("roboRIO/Drivetrain")
        
        # Get values with default of 0.0 if not found
        left_speed = drivetrain.getNumber("leftSpeed", 0.0)
        right_speed = drivetrain.getNumber("rightSpeed", 0.0)
        
        # Eli TODO: Add more data points you want to collect
        
        return {
            "left_speed": left_speed,
            "right_speed": right_speed,
            # Add your additional data points here
        }
    except:
        print("Error getting NetworkTables data")
        return None

def send_to_arduino(ser, data):
    """
    Send data to the Arduino over serial
    
    Eli TODO: Format the data how your Arduino expects it
    """
    if data is None:
        return False
    
    try:
        # Convert data to JSON string and add newline
        message = json.dumps(data) + "\n"
        
        # Send as bytes
        ser.write(message.encode())
        return True
    except:
        print("Error sending to Arduino")
        return False

def main():
    """Main program loop"""
    arduino = None
    connected_to_robot = False
    
    print("Starting FRC Button Box Data Relay")
    
    while True:
        # Try to connect to robot if not connected
        if not connected_to_robot:
            print("Connecting to robot at", ROBOT_IP)
            connected_to_robot = connect_to_networktables(ROBOT_IP)
            if connected_to_robot:
                print("Connected to robot!")
        
        # Try to connect to Arduino if not connected
        if arduino is None:
            try:
                arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
                print(f"Connected to Arduino on {ARDUINO_PORT}")
                time.sleep(2)  # Arduino resets on serial connection, give it time
            except:
                print(f"Could not connect to Arduino on {ARDUINO_PORT}")
                arduino = None
        
        # If connected to both robot and Arduino, get and send data
        if connected_to_robot and arduino is not None:
            robot_data = get_robot_data()
            send_success = send_to_arduino(arduino, robot_data)
            
            if not send_success:
                # If send failed, Arduino may be disconnected
                arduino = None
        
        # Check if still connected to robot
        connected_to_robot = NetworkTables.isConnected()
        
        # Eli TODO: Adjust this delay to control how often data is sent
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program terminated by user")
    except Exception as e:
        print(f"Unexpected error: {e}")