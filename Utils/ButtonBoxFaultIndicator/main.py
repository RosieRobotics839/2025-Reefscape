# main.py - The main application file

from networktables import NetworkTables
import serial
import time
import json

ROBOT_IP = "10.83.9.2"   # The Team's robot IP
ARDUINO_PORT = "COM2"    # Laptop Port
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
        endeffector = NetworkTables.getTable("roboRIO/EndEffector")
        gyro = NetworkTables.getTable("roboRIO/Gyro")
        elevator = NetworkTables.getTable("roboRIO/Elevator")
        arm = NetworkTables.getTable("roboRIO/Arm")
        climber = NetworkTables.getTable("roboRIO/Climber")
        funnel = NetworkTables.getTable("roboRIO/Funnel")
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
        encoder_is_reading = drivetrain.getBoolean("encoderIsReading", False)
        drive_motor_setup_done = drivetrain.getBoolean("driveMotorSetupDone", False)

        elevator_motor_temp_high = led.getBoolean("elevatorMotorTempHigh", False)
        elevator_pos_is_updating = elevator.getBoolean("elevatorPosIsUpdating", False)
        elevator_status = elevator.getBoolean("isCalibrated", False)

        arm_motor_temp_high = led.getBoolean("armMotorTempHigh", False)
        arm_pos_is_updating = arm.getBoolean("armPosIsUpdating", False)
        arm_status = arm.getBoolean("setupDone", False)

        effector_motor_temp_high = led.getBoolean("effectorMotorTempHigh", False)
        effector_pos_is_updating = endeffector.getBoolean("effectorPosIsUpdating", False)

        funnel_motor_temp_high = led.getBoolean("funnelMotorTempHigh", False)
        funnel_pos_is_updating = funnel.getBoolean("funnelPosIsUpdating", False)
        funnel_is_down = funnel.getBoolean("funnelIsDown", False)
        funnel_is_up = funnel.getBoolean("funnelIsUp", False)

        climber_motor_temp_high = led.getBoolean("climberMotorTempHigh", False)
        climber_pos_is_updating = climber.getBoolean("climberPosIsUpdating", False)
        climber_calibrated = climber.getBoolean("climberCalibrated", False)
        climber_is_in = climber.getBoolean("hasReachedInPos", False)
        climber_is_out = climber.getBoolean("hasReachedOutPos", False)
        
        return {
            "hasGamePiece": has_game_piece,
            "cam1IsConnected": cam1_connected,
            "cam2IsConnected": cam2_connected,
            "status":gyro_status,
            "fieldCentricDriving":field_centric_driving,
            "remainingMatchTime":remaining_match_time,
            "driveTrainMotorsTempHigh":drivetrain_motor_temp_high,
            "encoderIsReading":encoder_is_reading,
            "driveMotorSetupDone":drive_motor_setup_done,
            "elevatorMotorTempHigh":elevator_motor_temp_high,
            "elevatorPosIsUpdating":elevator_pos_is_updating,
            "isCalibrated":elevator_status,
            "armMotorTempHigh":arm_motor_temp_high,
            "armPosIsUpdating":arm_pos_is_updating,
            "setupDone":arm_status,
            "effectorMotorTempHigh":effector_motor_temp_high,
            "effectorPosIsUpdating":effector_pos_is_updating,
            "funnelMotorTempHigh":funnel_motor_temp_high,
            "funnelPosIsUpdating":funnel_pos_is_updating,
            "funnelIsDown":funnel_is_down,
            "funnelIsUp":funnel_is_up,
            "climberMotorTempHigh":climber_motor_temp_high,
            "climberPosIsUpdating":climber_pos_is_updating,
            "climberCalibrated":climber_calibrated,
            "hasReachedInPos":climber_is_in,
            "hasReachedOutPos":climber_is_out
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
        gamepiece = 2 if data.get("has_game_piece", True) else 1

        # Defining Cameras Value
        cam1_connected = data.get("cam1_connected", False)
        cam2_connected = data.get("cam2_connected", False)

        if cam1_connected and cam2_connected:
            cameras = 3 # Both Cameras connected
        elif cam1_connected and not cam2_connected:
            cameras = 1 # Cam 1 is connected but not cam 2
        elif cam2_connected and not cam1_connected:
            cameras = 2 # Cam 2 is connected but not cam 1
        else:
            cameras = 0 # Not recieving values
        
        # Defining Gyro Value
        field_centric_driving = data.get("field_centric_driving", False)
        gyro_hardware_good = data.get("gyro_status", False)

        if gyro_hardware_good and field_centric_driving:
            gyro_value = 3 # Gyro hardware is good and field centric is enabled
        if gyro_hardware_good and not field_centric_driving:
            gyro_value = 2 # Gyro hardware is good but field centric is disabled
        if not gyro_hardware_good:
            gyro_value = 1 # Gyro hardware is bad
        else:
            gyro_value = 0 # Not recieving values

        # Match Time
        remaining_match_time = data.get("remaining_match_time", 0.0)

        match_time = remaining_match_time

        # Defining Drive Train Value
        drivetrain_motor_temp_high = data.get("drivetrain_motor_temp_high", False)
        encoder_is_reading = data.get("encoder_is_reading", False)
        drive_motor_setup_done = data.get("drive_motor_setup_done", False)

        if not drivetrain_motor_temp_high and encoder_is_reading and drive_motor_setup_done:
            drivetrain = 4 # Everything is good
        elif data.get("drivetrain_motor_temp_high", False):
            drivetrain = 1 # Drive motors are overheating
        elif data.get("drive_motor_setup_done", False):
            drivetrain = 2 # Drive motor setup is not done
        elif data.get("encoder_is_reading", False):
            drivetrain = 3 # Swerve pod analog encoders are not reading
        else:
            drivetrain = 0 # Not recieving values

        # Defining Elevator Value
        elevator_motor_temp_high = data.get("elevator_motor_temp_high", False)
        elevator_pos_is_updating = data.get("elevator_pos_is_updating", False)
        elevator_status = data.get("elevator_status", False)

        if not elevator_motor_temp_high and elevator_pos_is_updating and elevator_status:
            elevator = 4 # Everything is good
        elif data.get("elevator_motor_temp_high", False):
            elevator = 1 # Elevator motor is overheating
        elif data.get("elevator_status", False):
            elevator = 2 # Elevator motor status is bad
        elif data.get("elevator_pos_is_updating", False):
            elevator = 3 # Elevator motor position isnt updating
        else:
            elevator = 0 # Not recieving values

        # Defining Arm Value
        arm_motor_temp_high = data.get("arm_motor_temp_high", False)
        arm_pos_is_updating = data.get("arm_pos_is_updating", False)
        arm_status = data.get("arm_status", False)

        if not arm_motor_temp_high and arm_pos_is_updating and arm_status:
            gantry_arm = 4 # Everything is good
        elif data.get("arm_motor_temp_high", False):
            gantry_arm = 1 # Arm motor is overheating
        elif data.get("arm_status", False):
            gantry_arm = 2 # Arm motor status is bad
        elif data.get("arm_pos_is_updating", False):
            gantry_arm = 3 # Arm motor position isnt updating
        else:
            gantry_arm = 0 # Not recieving values

        # Defining Intake Value
        effector_motor_temp_high = data.get("effector_motor_temp_high", False)
        effector_pos_is_updating = data.get("effector_pos_is_updating", False)

        if not effector_motor_temp_high and effector_pos_is_updating:
            intake = 3 # Everything is good
        elif data.get("effector_motor_temp_high", False):
            intake = 1 # EndEffector motor is overheating
        elif data.get("effector_pos_is_updating", False):
            intake = 2 # EndEffector motor position isnt updating
        else:
            intake = 0 # Not recieving values

        # Defining Funnel Value
        funnel_motor_temp_high = data.get("funnel_motor_temp_high", False)
        funnel_pos_is_updating = data.get("funnel_pos_is_updating", False)
        funnel_is_down = data.get("funnel_is_down", False)
        funnel_is_up = data.get("funnel_is_up", False)

        if not funnel_motor_temp_high and funnel_pos_is_updating and funnel_is_down:
            funnel = 3 # Everything is good, funnel is down
        elif not funnel_motor_temp_high and funnel_pos_is_updating and funnel_is_up:
            funnel = 4 # Everything is good, funnel is up
        elif data.get("funnel_motor_temp_high", False):
            funnel = 1 # Funnel motor is overheating
        elif data.get("funnel_pos_is_updating", False):
            funnel = 2 # Funnel motor position isnt updating
        else:
            funnel = 0 # Not recieving values

        # Defining Climber Value
        climber_motor_temp_high = data.get("climber_motor_temp_high", False)
        climber_pos_is_updating = data.get("climber_pos_is_updating", False)
        climber_calibrated = data.get("climber_calibrated", False)
        climber_is_in = data.get("climber_is_in", False)
        climber_is_out = data.get("climber_is_out", False)

        if not climber_motor_temp_high and climber_pos_is_updating and climber_calibrated and climber_is_in:
            climber = 4 # Everything is good, climber is in
        elif not climber_motor_temp_high and climber_pos_is_updating and climber_calibrated and climber_is_out:
            climber = 5 # Everything is good, climber is out
        elif data.get("climber_motor_temp_high", False):
            climber = 1 # Climber motor is overheating
        elif data.get("climber_calibrated", False):
            climber = 2 # Climber motor isnt calibrated
        elif data.get("climber_pos_is_updating", False):
            climber = 3 # Climber motor position isnt updating
        else:
            climber = 0 # Not recieving values

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