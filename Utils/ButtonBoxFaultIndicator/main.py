# main.py - The main application file

from networktables import NetworkTables
import serial
import time
import json

ROBOT_IP = "10.8.39.2"   # The Team's robot IP
ARDUINO_PORT = "COM4"    # Laptop Port
BAUD_RATE = "115200"     # Arduino baud rate

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

    """
    try:
        # Getting subsystem data from network tables
        drivetrain = NetworkTables.getTable("roboRIO/Drivetrain")
        endeffector = NetworkTables.getTable("roboRIO/EndEffector/Effector")
        gyro = NetworkTables.getTable("roboRIO/Gyro")
        elevator = NetworkTables.getTable("roboRIO/Elevator")
        arm_debug = NetworkTables.getTable("roboRIO/Arm/table/debug")
        climber = NetworkTables.getTable("roboRIO/Climber")
        funnel = NetworkTables.getTable("roboRIO/Funnel/table")
        vision = NetworkTables.getTable("roboRIO/Vision")
        flightstick = NetworkTables.getTable("roboRIO/FlightStick")
        led = NetworkTables.getTable("roboRIO/LED")
        
        # Getting specific values/booleans from network tables subsystems
        has_game_piece = endeffector.getBoolean("hasGamePiece", False)

        cam1_connected = vision.getBoolean("cam1IsConnected", False)
        cam2_connected = vision.getBoolean("cam2IsConnected", False)

        gyro_status = gyro.getBoolean("status", False)
        field_centric_driving = flightstick.getBoolean("fieldCentricDriving", False)

        remaining_match_time = led.getNumber("remainingMatchTime", 7500)

        drivetrain_motor_temp_high = led.getBoolean("driveTrainMotorsTempHigh", False)
        drive_motor_setup_done = drivetrain.getBoolean("driveMotorSetupDone", False)

        elevator_motor_temp_high = led.getBoolean("elevatorMotorTempHigh", False)
        elevator_status = elevator.getBoolean("isCalibrated", False)

        arm_motor_temp_high = led.getBoolean("armMotorTempHigh", False)
        arm_status = arm_debug.getBoolean("setupDone", False)

        effector_motor_temp_high = led.getBoolean("effectorMotorTempHigh", False)

        funnel_motor_temp_high = led.getBoolean("funnelMotorTempHigh", False)
        funnel_is_down = funnel.getBoolean("funnelIsDown", False)
        funnel_is_up = funnel.getBoolean("funnelIsUp", False)

        climber_motor_temp_high = led.getBoolean("climberMotorTempHigh", False)
        climber_calibrated = climber.getBoolean("climberCalibrated", False)
        climber_is_in = climber.getBoolean("hasReachedInPos", False)
        climber_is_out = climber.getBoolean("hasReachedOutPos", False)
        
        return {
            "hasGamePiece": has_game_piece,
            "cam1IsConnected": cam1_connected,
            "cam2IsConnected": cam2_connected,
            "gyroStatus":gyro_status,
            "fieldCentricDriving":field_centric_driving,
            "remainingMatchTime":remaining_match_time,
            "driveTrainMotorsTempHigh":drivetrain_motor_temp_high,
            "driveMotorSetupDone":drive_motor_setup_done,
            "elevatorMotorTempHigh":elevator_motor_temp_high,
            "elevatorStatus":elevator_status,
            "armMotorTempHigh":arm_motor_temp_high,
            "armStatus":arm_status,
            "effectorMotorTempHigh":effector_motor_temp_high,
            "funnelMotorTempHigh":funnel_motor_temp_high,
            "funnelIsDown":funnel_is_down,
            "funnelIsUp":funnel_is_up,
            "climberMotorTempHigh":climber_motor_temp_high,
            "climberCalibrated":climber_calibrated,
            "climberIsIn":climber_is_in,
            "climberIsOut":climber_is_out
        }
    except:
        print("Error getting NetworkTables data")
        return None

def send_to_arduino(ser, data):
    """
    Send data to the Arduino over serial
    
    """
    if data is None:
        return False
    
    try:

        # Gamepiece Value
        has_game_piece =  data.get("hasGamePiece", False)

        if has_game_piece:
            gamepiece = 2
        else:
            gamepiece = 1

        # Defining Cameras Value
        cam1_connected = data.get("cam1IsConnected", False)
        cam2_connected = data.get("cam2IsConnected", False)

        if cam1_connected and cam2_connected:
            cameras = 3 # Both Cameras connected
        elif cam2_connected and not cam1_connected:
            cameras = 2 # Cam 2 is connected but not cam 1
        elif cam1_connected and not cam2_connected:
            cameras = 1 # Cam 1 is connected but not cam 2
        else:
            cameras = 0 # Not receiving values
        
        # Defining Gyro Value
        field_centric_driving = data.get("fieldCentricDriving", False)
        gyro_hardware_good = data.get("gyroStatus", False)

        if gyro_hardware_good and field_centric_driving:
            gyro_value = 3 # Gyro hardware is good and field centric is enabled
        elif gyro_hardware_good and not field_centric_driving:
            gyro_value = 2 # Gyro hardware is good but field centric is disabled
        elif not gyro_hardware_good:
            gyro_value = 1 # Gyro hardware is bad
        else:
            gyro_value = 0 # Not receiving values

        # Match Time
        remaining_match_time = data.get("remainingMatchTime", 0.0)

        match_time = remaining_match_time

        # Defining Drive Train Value
        drivetrain_motor_temp_high = data.get("driveTrainMotorsTempHigh", False)
        drive_motor_setup_done = data.get("driveMotorSetupDone", False)

        if not drivetrain_motor_temp_high and drive_motor_setup_done:
            drivetrain = 3 # Everything is good
        elif data.get("driveMotorSetupDone", False):
            drivetrain = 2 # Drive motor setup is not done
        elif data.get("driveTrainMotorsTempHigh", False):
            drivetrain = 1 # Drive motors are overheating
        else:
            drivetrain = 0 # Not receiving values

        # Defining Elevator Value
        elevator_motor_temp_high = data.get("elevatorMotorTempHigh", False)
        elevator_status = data.get("elevatorStatus", False)

        if not elevator_motor_temp_high and elevator_status:
            elevator = 3 # Everything is good
        elif data.get("elevatorStatus", False):
            elevator = 2 # Elevator motor status is bad
        elif data.get("elevatorMotorTempHigh", False):
            elevator = 1 # Elevator motor is overheating
        else:
            elevator = 0 # Not receiving values

        # Defining Arm Value
        arm_motor_temp_high = data.get("armMotorTempHigh", False)
        arm_status = data.get("armStatus", False)

        if not arm_motor_temp_high  and arm_status:
            gantry_arm = 3 # Everything is good
        elif data.get("armStatus", False):
            gantry_arm = 2 # Arm motor status is bad
        elif data.get("armMotorTempHigh", False):
            gantry_arm = 1 # Arm motor is overheating
        else:
            gantry_arm = 0 # Not receiving values

        # Defining Intake Value
        effector_motor_temp_high = data.get("effectorMotorTempHigh", False)

        if not effector_motor_temp_high:
            intake = 2 # Everything is good
        elif data.get("effectorMotorTempHigh", False):
            intake = 1 # EndEffector motor is overheating
        else:
            intake = 0 # Not receiving values

        # Defining Funnel Value
        funnel_motor_temp_high = data.get("funnelMotorTempHigh", False)
        funnel_is_down = data.get("funnelIsDown", False)
        funnel_is_up = data.get("funnelIsUp", False)

        if not funnel_motor_temp_high and funnel_is_down:
            funnel = 3 # Everything is good, funnel is down
        elif not funnel_motor_temp_high and funnel_is_up:
            funnel = 2 # Everything is good, funnel is up
        elif data.get("funnelMotorTempHigh", False):
            funnel = 1 # Funnel motor is overheating
        else:
            funnel = 0 # Not receiving values

        # Defining Climber Value
        climber_motor_temp_high = data.get("climberMotorTempHigh", False)
        climber_calibrated = data.get("climberCalibrated", False)
        climber_is_in = data.get("climberIsIn", False)
        climber_is_out = data.get("climberIsOut", False)

        if not climber_motor_temp_high and climber_calibrated and climber_is_in:
            climber = 4 # Everything is good, climber is in
        elif not climber_motor_temp_high and climber_calibrated and climber_is_out:
            climber = 3 # Everything is good, climber is out
        elif data.get("climberCalibrated", False):
            climber = 2 # Climber motor isnt calibrated
        elif data.get("climberMotorTempHigh", False):
            climber = 1 # Climber motor is overheating
        else:
            climber = 0 # Not receiving values

        # Format the final string with the required pattern
        message = f"<{gamepiece}, {cameras}, {gyro_value}, {match_time}, {drivetrain}, {elevator}, {gantry_arm}, {intake}, {funnel}, {climber}>\n"
        
        # Send as bytes
        ser.write(message.encode())
        print(f"Sent to Arduino: {message.strip()}")  # For debugging
        return True
    except Exception as e:
        print(f"Error sending to Arduino: {e}")
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
        
        time.sleep(1)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Program terminated by user")
    except Exception as e:
        print(f"Unexpected error: {e}")