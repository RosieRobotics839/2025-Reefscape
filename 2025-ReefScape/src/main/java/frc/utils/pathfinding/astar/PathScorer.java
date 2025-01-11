// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils.pathfinding.astar;

import frc.robot.subsystems.Autonomous;
import frc.utils.VectorUtils;

/** Add your docs here. */
public class PathScorer implements Scorer<FieldPose> {
    @Override
    public double computeCost(FieldPose from, FieldPose to) {
        if(from == null || to == null){
            return 0;
        }

        double R=0;
        
        // Penalize route for passing through a static obstacle
        if (PathfindingUtils.PathIntersectsPolygon(
            from.getPose().getTranslation(),
            to.getPose().getTranslation(),
            Autonomous.staticObstacles)
        ){
            R+=999999;
        };

        R += VectorUtils.poseDiff(to.getPose(),from.getPose()).getTranslation().getNorm();
        return R;
    }
    public PathScorer(){
    }
}
