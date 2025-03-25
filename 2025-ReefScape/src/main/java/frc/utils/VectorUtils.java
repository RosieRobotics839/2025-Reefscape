// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;
import java.util.Optional;

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
    return vectorInDirectionOf(pose.getTranslation(), magnitude);
  }

  public static Translation2d vectorInDirectionOf(Translation2d pose, double magnitude){
    // WPILIB Rotation2d reports an error if magnitude is less than a hardcoded unitless value of 1e-6. (what if we were measuring in astronomical units?) :(
    if (pose.getNorm() < 1e-6 || Math.abs(magnitude) < 1e-6){
      return new Translation2d(0, 0);
    } else {
      return new Translation2d(magnitude, pose.getAngle());      
    }
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
    Rotation2d angle_out;

    if (r_in < deadband){
      r_out = 0;
    } else if (r_in < sickband){
      r_out = (r_in - deadband) / (sickband - deadband) * sickband;
    } else {
      r_out = r_in;
    }
    if (r_out == 0) {
      angle_out = new Rotation2d(0);
    } else {
      angle_out = vector.getAngle();
    }
    return new Translation2d(r_out, angle_out);
  }

  /**
   * Returns the nearest point on a line segment to a pose2d object.
   *
   * @param point The point to determine the nearest point to.
   * @param lineStart The starting point of the line segment.
   * @param lineEnd The ending point of the line segment.
   * @return The point on the line which is nearest to the point.
   */
  public static Translation2d nearestPointOnLine(Translation2d point, Translation2d lineStart, Translation2d lineEnd){
      double dx = lineEnd.getX() - lineStart.getX();
      double dy = lineEnd.getY() - lineStart.getY();

      if (dx == 0 && dy == 0) {
          // Line is a point, return the line's point.
          return lineStart;
      }

      double t = ((point.getX() - lineStart.getX()) * dx + (point.getY() - lineStart.getY()) * dy) / (dx * dx + dy * dy);
      
      // Range limit to only allow points on the line segment.
      t = Math.max(0,Math.min(1,t));

      double nearestX = lineStart.getX() + t * dx;
      double nearestY = lineStart.getY() + t * dy;

      return new Translation2d(nearestX, nearestY);
  }

  /**
   * Determines the intersection point of two line segments.
   * Reference: https://paulbourke.net/geometry/pointlineplane/
   * @param a The starting point of the first line segment.
   * @param b The ending point of the first line segment.
   * @param c The starting point of the second line segment.
   * @param d The ending point of the second line segment.
   * @return The intersection point, or empty if they do not intersect within the segments.
   */
  public static Optional<Translation2d> findIntersection(Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    // Check if the lines are of length 0
    if ((a.getX() == b.getX() && a.getY() == b.getY()) || (c.getX() == d.getX() && c.getY() == d.getY())) {
      return Optional.empty();
    }
  
    // Calculate the denominator of the intersection point formula
    double denominator = ((d.getY() - c.getY()) * (b.getX() - a.getX()) - (d.getX() - c.getX()) * (b.getY() - a.getY()));
    // If the denominator is zero, the lines are parallel
  
    if (denominator == 0) {
        return Optional.empty();
    }
  
    // Calculate the parameters t and u
    double tNumerator = ((d.getX()-c.getX())*(a.getY()-c.getY())-(d.getY()-c.getY())*(a.getX()-c.getX()));
    double uNumerator = ((b.getX()-a.getX())*(a.getY()-c.getY())-(b.getY()-a.getY())*(a.getX()-c.getX()));
  
    double t = tNumerator / denominator;
    double u = uNumerator / denominator;
  
    // Check if intersection is outside the line segments
    if (t < 0 || t > 1 || u < 0 || u > 1){
        return Optional.empty();
    }
  
    // Calculate the intersection point coordinates
    double intersectionX = a.getX() + t * (b.getX() - a.getX());
    double intersectionY = a.getY() + t * (b.getY() - a.getY());
    return Optional.of(new Translation2d(intersectionX, intersectionY));
  }
}