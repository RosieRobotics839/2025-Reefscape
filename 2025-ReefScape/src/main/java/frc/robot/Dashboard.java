package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class Dashboard{

    static Dashboard instance = new Dashboard();
    public static Dashboard getInstance(){
        return instance;
    }

    int m_numAutoPoses = 6;
    List<Pose2d> m_autoPoses = new ArrayList<Pose2d>();
    {
        var horiz_margin=0.2;
        var vert_margin = 1.1;
        for (int i=0; i<m_numAutoPoses; i++){
        m_autoPoses.add(new Pose2d(Vision.getInstance().aprilTagFieldLayout.getFieldLength()*(horiz_margin+(i/(double)m_numAutoPoses)*(1-2*horiz_margin)),Vision.getInstance().aprilTagFieldLayout.getFieldWidth()*vert_margin, new Rotation2d(Math.PI/4)));
        }
        PoseEstimator.getInstance().m_field.getObject("AutoPoses").setPoses(m_autoPoses);
    }
    
/*
 * SCORE command to drive to the reef and score should look like this
 * Score.SS.Left.4     The period are not required but help seperate the arguments for accessibility
 * ENSURE THAT THE SCORING LEVEL IS THE LAST CHAR IN THE STRING
 * 0 = trough, 4 = highest level
 * 
 * GET or SOURCE command to drive to the source to get a coral
 * Get.left || Source.right
 * 
 * WAIT command to wait a time in integer seconds
 * Wait.3
 * ENSURE THAT THE WAIT TIME IS THE LAST CHAR IN THE STRING
 * 
 * STORE command and RETURN command to store and return to a stored pose
 * Store
 * Return
 * Neither have any extra arguments
 */
}