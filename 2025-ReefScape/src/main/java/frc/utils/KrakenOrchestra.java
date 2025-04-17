package frc.utils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class KrakenOrchestra extends SubsystemBase {

    static NetworkTable table = NetworkTableInstance.getDefault().getTable("roboRIO/Orchestra");

    private static KrakenOrchestra instance = new KrakenOrchestra();

    public static KrakenOrchestra getInstance(){
        return instance;
    }

    public Orchestra m_orchestra = new Orchestra();

    private List<TalonFX> availableMotors = new ArrayList<>();
    private String currentSong = "";
    private boolean orchestraReady = false;
    
    // Song track configuration - maps song filenames to track assignments
    private Map<String, int[]> songTrackMap = new HashMap<>();
    
    // Current track assignments for motors
    private Map<TalonFX, Integer> currentMotorTrackMap = new HashMap<>();
    
    BooleanPublisher nt_orchestraIsPlaying = table.getBooleanTopic("orchestraIsPlaying").publish();
    BooleanPublisher nt_orchestraIsReady = table.getBooleanTopic("orchestraIsReady").publish();
    IntegerPublisher nt_instrumentCount = table.getIntegerTopic("instrumentCount").publish();
    IntegerPublisher nt_playStatus = table.getIntegerTopic("playStatus").publish();
    IntegerPublisher nt_stopStatus = table.getIntegerTopic("stopStatus").publish();
    IntegerPublisher nt_loadStatus = table.getIntegerTopic("loadStatus").publish();
    StringPublisher nt_currentFile = table.getStringTopic("currentSongPlaying").publish();
    StringPublisher nt_trackAssignments = table.getStringTopic("trackAssignments").publish();
    DoublePublisher nt_fileTimeStamp = table.getDoubleTopic("fileTimeStamp").publish();

    private KrakenOrchestra() {
        // Initialize song track assignments
        initializeSongTracks();
        
        // Initialize orchestra with available Kraken motors
        initializeMotors();
    }

    private void initializeSongTracks() {
        // Define preferred tracks for each song
        // Format: songTrackMap.put("filename.chrp", new int[]{track1, track2, ...});
        
        // Default tracks (0-5) for most songs
        songTrackMap.put("song10.chrp", new int[]{0, 0, 0, 0, 0, 0});
        songTrackMap.put("congrats.chrp", new int[]{0, 0, 0, 0, 0, 0});
        songTrackMap.put("underthesea.chrp", new int[]{0, 0, 0, 0, 0, 0});
        
        // Songs with specific melody tracks prioritized
        songTrackMap.put("undertheseamelodies.chrp", new int[]{4, 5, 6, 7, 8, 9});
        songTrackMap.put("spongebobopening.chrp", new int[]{0, 0, 0, 0, 0, 0});
        songTrackMap.put("jetsons.chrp", new int[]{1, 2, 5, 9, 10, 12});
        songTrackMap.put("wellerman.chrp", new int[]{0, 0, 0, 0, 0, 0});
        songTrackMap.put("wellermans.chrp", new int[]{0, 0, 0, 1, 1, 1});
        songTrackMap.put("jeopardy.chrp", new int[]{0, 1, 1, 2, 3, 3});
        songTrackMap.put("song2.chrp", new int[]{0, 0, 0, 0, 0, 0});
        songTrackMap.put("song5.chrp", new int[]{0, 0, 0, 0, 0, 0});
        songTrackMap.put("eyetiger.chrp", new int[]{0, 1, 2, 4, 5, 6});
        songTrackMap.put("hespirate.chrp", new int[]{0, 0, 0, 1, 1, 1});
        songTrackMap.put("updown.chrp", new int[]{0, 0, 0, 0, 0, 0});
        songTrackMap.put("davyjones.chrp", new int[]{0, 0, 0, 0, 0, 0});
        songTrackMap.put("loudsound.chrp", new int[]{0, 1, 2, 3, 4, 5});
    }

    private void initializeMotors() {
        // Clear any existing motors
        availableMotors.clear();
        
        try {
            // Collect all available TalonFX motor controllers
            List<TalonFX> motors = new ArrayList<>();
            
            // Add Arm motor if available
            if (Arm.getInstance().m_motor.motor_talon != null) {
                motors.add(Arm.getInstance().m_motor.motor_talon);
            }
            
            // Add Elevator motor if available
            if (Elevator.getInstance().m_EleMotor.motor_talon != null) {
                motors.add(Elevator.getInstance().m_EleMotor.motor_talon);
            }
            
            // Add DriveTrain motors if available
            DriveTrain dt = DriveTrain.getInstance();
            if (dt.frontLeft.m_motorDrive.motor_talon != null) {
                motors.add(dt.frontLeft.m_motorDrive.motor_talon);
            }
            if (dt.frontRight.m_motorDrive.motor_talon != null) {
                motors.add(dt.frontRight.m_motorDrive.motor_talon);
            }
            if (dt.rearLeft.m_motorDrive.motor_talon != null) {
                motors.add(dt.rearLeft.m_motorDrive.motor_talon);
            }
            if (dt.rearRight.m_motorDrive.motor_talon != null) {
                motors.add(dt.rearRight.m_motorDrive.motor_talon);
            }
            
            // Save the available motors
            availableMotors = motors;
            
            // Configure all motors to allow music during disabled
            for (TalonFX motor : availableMotors) {
                var audioConfig = new com.ctre.phoenix6.configs.AudioConfigs();
                audioConfig.withAllowMusicDurDisable(true);
                motor.getConfigurator().apply(audioConfig);
            }
            
            orchestraReady = !availableMotors.isEmpty();
        } catch (Exception e) {
            orchestraReady = false;
            System.out.println("Error initializing motors: " + e.getMessage());
        }
    }

    private void assignTracksForSong(String songFilepath) {
        // Clear the orchestra and track assignments
        m_orchestra = new Orchestra();
        currentMotorTrackMap.clear();
        
        // Get the song filename
        String filename = extractFilename(songFilepath);
        
        // Get the track assignments for this song, or use sequential tracks if none exists
        int[] tracksToPlay;
        int[] configuredTracks = songTrackMap.get(filename);
        int motorCount = availableMotors.size();
        
        if (configuredTracks != null && configuredTracks.length > 0) {
            // Use configured tracks, limited by available motors
            tracksToPlay = new int[Math.min(motorCount, configuredTracks.length)];
            for (int i = 0; i < tracksToPlay.length; i++) {
                tracksToPlay[i] = configuredTracks[i];
            }
        } else {
            // Default to sequential tracks (0, 1, 2, ...)
            tracksToPlay = new int[motorCount];
            for (int i = 0; i < motorCount; i++) {
                tracksToPlay[i] = i;
            }
        }
        
        // Assign the tracks to motors
        int successCount = 0;
        StringBuilder trackAssignments = new StringBuilder("Track Assignments: ");
        
        for (int i = 0; i < tracksToPlay.length; i++) {
            TalonFX motor = availableMotors.get(i);
            int trackId = tracksToPlay[i];
            
            StatusCode addStatus = m_orchestra.addInstrument(motor, trackId);
            
            if (addStatus.isOK()) {
                successCount++;
                currentMotorTrackMap.put(motor, trackId);
                trackAssignments.append("Motor").append(i).append("->Track").append(trackId).append(" ");
            }
        }
        
        orchestraReady = successCount > 0;
        nt_trackAssignments.set(trackAssignments.toString());
    }

    private String extractFilename(String filepath) {
        if (filepath == null || filepath.isEmpty()) {
            return "";
        }
        
        // Get just the filename from a filepath
        int lastSlashIndex = filepath.lastIndexOf('/');
        return lastSlashIndex == -1 ? filepath : filepath.substring(lastSlashIndex + 1);
    }

    // Playing Music 
    public void playMusic(String filepath) {
        if (filepath == null || filepath.isEmpty()) {
            return;
        }
        
        // Extract just the filename for display
        String filename = extractFilename(filepath);

        // If we're already playing this song, don't restart it
        if (isPlaying() && filename.equals(extractFilename(currentSong))) {
            return;
        }
        
        if (!orchestraReady) {
            initializeMotors();
            if (!orchestraReady) {
                return;
            }
        }

        // Assign tracks based on the song configuration
        assignTracksForSong(filepath);
        
        currentSong = filepath;
        stopMusic(); // Stop any currently playing music

        StatusCode loadStatus = m_orchestra.loadMusic(filepath);
        nt_loadStatus.set(loadStatus.value);

        if (loadStatus.isOK()) {
            StatusCode playStatus = m_orchestra.play();
            nt_playStatus.set(playStatus.value);
            System.out.println("Playing music: " + filepath);
        } else {
            nt_playStatus.set(loadStatus.value);
            System.out.println("Failed to load music: " + filepath);
        }
    
        // Update Network Table with current song
        nt_currentFile.set(filename);
    }

    public void stopMusic(){
        if (orchestraReady) {
            StatusCode stopStatus = m_orchestra.stop();
            nt_stopStatus.set(stopStatus.value);
        }
    }

    public boolean isPlaying(){
        return m_orchestra.isPlaying() && orchestraReady;
    }

    public boolean isReady() {
        return orchestraReady;
    }

    public double getTimeStamp() {
        return m_orchestra.getCurrentTime();
    }
    
    // Get the current motor-track assignments
    public Map<TalonFX, Integer> getCurrentTrackAssignments() {
        return new HashMap<>(currentMotorTrackMap);
    }
    
    // Add or update a song's track assignments
    public void updateSongTracks(String songFilename, int[] trackAssignments) {
        songTrackMap.put(extractFilename(songFilename), trackAssignments);
    }

    @Override
    public void periodic() {
        nt_orchestraIsPlaying.set(isPlaying());
        nt_orchestraIsReady.set(orchestraReady);
        nt_instrumentCount.set(availableMotors.size());
        nt_fileTimeStamp.set(getTimeStamp());

        // If motors have been initialized since our last check, try initializing instruments again
        if (!orchestraReady) {
            // Check if the motors we need are now available
            boolean motorsAvailable = false;
            
            if (Arm.getInstance().m_motor.isSetupDone() && 
                Elevator.getInstance().m_EleMotor.isSetupDone() &&
                DriveTrain.getInstance().isSetupDone()) {
                
                motorsAvailable = true;
            }
            
            if (motorsAvailable) {
                initializeMotors();
            }
        }
    }
}