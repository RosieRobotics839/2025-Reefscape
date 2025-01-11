// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class VectorUtils {
    
  // Calculates the minimum difference betweeen two angles with a result between -180 and +180 degrees.
    public static double angleDifference (double a1, double a2){
      return (((a1 % (2*Math.PI)) - (a2 % (2*Math.PI)) + 3*Math.PI) % (2*Math.PI) - Math.PI);
    }

    public static boolean isNear(Pose2d pose1, Pose2d pose2, double toleranceMeters){
      if (pose1 == null || pose2 == null){
        return false;
      }
      Pose2d diff = poseDiff(pose1,pose2);
      return diff.getTranslation().getNorm() < toleranceMeters; 
    }

    public static boolean isNear(Pose2d pose1, Pose2d pose2, double toleranceMeters, double toleranceRadians){
      if (pose1 == null || pose2 == null){
        return false;
      }
      Pose2d diff = poseDiff(pose1,pose2);
      return diff.getTranslation().getNorm() < toleranceMeters && Math.abs(diff.getRotation().getRadians()) < toleranceRadians; 
    }

    public static Pose2d poseAdd(Pose2d pose1, Pose2d pose2){
      return new Pose2d(
        pose1.getX()+pose2.getX(),
        pose1.getY()+pose2.getY(),
        pose1.getRotation().plus(pose2.getRotation())
      );
    }
    
    public static Translation2d vectorInDirectionOf(Pose2d pose, double magnitude){
      return new Translation2d(magnitude, pose.getTranslation().getAngle());
    }

    public static boolean isInDistanceAndAngle(Pose2d pose, Pose2d target, double meters, double radiansFromTargetFace){
      Pose2d diff = poseDiff(pose,target); // offset between
      if (diff.getTranslation().getNorm() > meters){
        return false;
      }
      double targetAngle;
      double angleMax;
      double angleMin;
      if (target.getRotation() == null){
        targetAngle = 0;
        angleMax = 2*Math.PI;
        angleMin = 0;
      } else {
        targetAngle = Math.PI+target.getRotation().getRadians();
        angleMax = radiansFromTargetFace;
        angleMin = -radiansFromTargetFace;
      }
      if (VectorUtils.angleDifference(diff.getTranslation().getAngle().getRadians(),targetAngle) < angleMin){
        return false;
      } else 
      if (VectorUtils.angleDifference(diff.getTranslation().getAngle().getRadians(),targetAngle) > angleMax){
        return false;
      }

      return true;

    }
    public static Pose2d closestPoseAtDistance(Pose2d pose, Pose2d target, double meters, double radiansFromTargetFace){
      Pose2d diff = poseDiff(pose,target); // offset between
      Translation2d vector = vectorInDirectionOf(diff, meters);
      double targetAngle;
      double angleMax;
      double angleMin;
      if (target.getRotation() == null){
        targetAngle = 0;
        angleMax = 2*Math.PI;
        angleMin = 0;
      } else {
        targetAngle = Math.PI+target.getRotation().getRadians();
        angleMax = targetAngle+radiansFromTargetFace;
        angleMin = targetAngle-radiansFromTargetFace;
      }
   
      if (VectorUtils.angleDifference(vector.getAngle().getRadians(),targetAngle) < -radiansFromTargetFace){
        vector = new Translation2d(meters, new Rotation2d(angleMin));
      } else 
      if (VectorUtils.angleDifference(vector.getAngle().getRadians(),targetAngle) > radiansFromTargetFace){
        vector = new Translation2d(meters, new Rotation2d(angleMax));
      }

      return new Pose2d(target.getTranslation().plus(vector),null);

    }

    public static Pose2d poseDiff(Pose2d pose1, Pose2d pose2){
      Rotation2d rotation;
      if (pose1.getRotation() != null && pose2.getRotation() != null){
        rotation = new Rotation2d(angleDifference(
              pose1.getRotation().getRadians(),
              pose2.getRotation().getRadians()
        ));
      } else {
        rotation = new Rotation2d(0);
      }

      return new Pose2d(pose1.getX()-pose2.getX(), pose1.getY()-pose2.getY(), rotation);
    }

    public static Pose2d desaturateXY(Pose2d input, double maximum){
      double ratio = maximum/Math.max(maximum,input.getTranslation().getNorm());
      return new Pose2d(input.getTranslation().times(ratio), input.getRotation());
    } 

    public static double SRSS(double... args){
      double sum = 0;
      for (double x : args){
        sum += Math.pow(x,2);
      }
      return Math.sqrt(sum);
    }
    public static Translation2d deadband(Translation2d vector, double deadband, double sickband){

      double r_in = vector.getNorm();
      double r_out;
  
      if (r_in < deadband){
        r_out = 0;
      } else if (r_in < sickband){
        r_out = (r_in - deadband) / (sickband - deadband) * sickband;
      } else {
        r_out = r_in;
      }
  
      return new Translation2d(r_out, vector.getAngle());
    }
}
