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
            "climberIsIn":climber_is_in
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

        # GAMEPIECE VALUE (1: No gamepiece, 2: Has gamepiece)
        has_game_piece =  data.get("hasGamePiece")

        if has_game_piece:
            gamepiece = 2
        else:
            gamepiece = 1

        # CAMERA VALUE (1: Cam 1 connected but not Cam 2, 2: Cam 2 connected but not Cam 1, 3: Both cameras connected)
        cam1_connected = data.get("cam1IsConnected")
        cam2_connected = data.get("cam2IsConnected")
        
        if cam1_connected is not None and cam2_connected is not None:
            if not cam2_connected and cam1_connected:
                cameras = 1 # Cam 1 is connected but not cam 2
            elif not cam1_connected and cam2_connected:
                cameras = 2 # Cam 2 is connected but not cam 1
            else:
                cameras = 3 # Both Cameras connected
        else:
            cameras = 0 # Not receiving values
        
        # GYRO VALUE (1: Hardware bad, 2: field centric disabled, 3: field centric enabled)
        field_centric_driving = data.get("fieldCentricDriving")
        gyro_hardware_good = data.get("gyroStatus")

        if gyro_hardware_good is not None:
            if not gyro_hardware_good:
                gyro_value = 1  # Gyro hardware is bad
            elif not field_centric_driving:
                gyro_value = 2  # Gyro hardware is good, but field centric is disabled
            elif field_centric_driving:
                gyro_value = 3  # Gyro hardware is good, and field centric is enabled
        else:
            gyro_value = 0 # Not receiving values

        # MATCH TIME
        remaining_match_time = data.get("remainingMatchTime")

        match_time = int(remaining_match_time)

        # DRIVETRAIN VALUE (1: overheated, 2: not calibrated, 3: all good)
        drivetrain_motor_temp_high = data.get("driveTrainMotorsTempHigh")
        drive_motor_setup_done = data.get("driveMotorSetupDone")

        if drivetrain_motor_temp_high is not None and drive_motor_setup_done is not None:
            if drivetrain_motor_temp_high:
                drivetrain = 1  # Overheated
            elif not drive_motor_setup_done:
                drivetrain = 2  # Not calibrated
            else:
                drivetrain = 3  # Everything correct
        else:
            drivetrain = 0  # Not receiving values

        # ELEVATOR VALUE (1: overheated, 2: not calibrated, 3: all good)
        elevator_motor_temp_high = data.get("elevatorMotorTempHigh")
        elevator_status = data.get("elevatorStatus")

        if elevator_motor_temp_high is not None and elevator_status is not None:
            if elevator_motor_temp_high:
                elevator = 1  # Overheated
            elif not elevator_status:
                elevator = 2  # Not calibrated
            else:
                elevator = 3  # Everything correct
        else:
            elevator = 0  # Not receiving values

        # ARM VALUE (1: overheated, 2: not calibrated, 3: all good)
        arm_motor_temp_high = data.get("armMotorTempHigh")
        arm_status = data.get("armStatus")

        if arm_motor_temp_high is not None and arm_status is not None:
            if arm_motor_temp_high:
                gantry_arm = 1  # Overheated
            elif not arm_status:
                gantry_arm = 2  # Not calibrated
            else:
                gantry_arm = 3  # Everything correct
        else:
            gantry_arm = 0 # Not receiving values

        # INTAKE VALUE (1: overheated, 2: all good)
        effector_motor_temp_high = data.get("effectorMotorTempHigh")

        if effector_motor_temp_high is not None:
            if effector_motor_temp_high:
                intake = 1  # Overheated
            else:
                intake = 2  # Everything correct
        else:
            intake = 0  # Not receiving values

        # FUNNEL VALUE (1: overheated, 2: funnel up, 3: funnel down)
        funnel_motor_temp_high = data.get("funnelMotorTempHigh")
        funnel_is_down = data.get("funnelIsDown")
        funnel_is_up = data.get("funnelIsUp")

        if funnel_motor_temp_high is not None:
            if funnel_motor_temp_high:
                funnel = 1  # Overheated
            elif funnel_is_up:
                funnel = 2  # Funnel up, everything good
            elif funnel_is_down:
                funnel = 3  # Funnel down, everything good
            else:
                funnel = 0 # No current funnel state detected
        else:
            funnel = 0  # Not receiving values

        # CLIMBER VALUE (1: overheated, 2: Not calibrated, 3: climber out, 4: climber in)
        climber_motor_temp_high = data.get("climberMotorTempHigh")
        climber_calibrated = data.get("climberCalibrated")
        climber_is_in = data.get("climberIsIn")

        if climber_motor_temp_high is not None and climber_calibrated is not None:
            if climber_motor_temp_high:
                climber = 1  # Climber motor is overheating
            elif not climber_calibrated:
                climber = 2  # Climber motor isn’t calibrated
            elif not climber_is_in:
                climber = 3  # Everything is good, climber is out
            elif climber_is_in:
                climber = 4  # Everything is good, climber is in
            else:
                climber = 0
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