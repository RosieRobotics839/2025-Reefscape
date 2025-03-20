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

        var fromObstacle = PathfindingUtils.PointInConvexPolygons(from.getPose().getTranslation(), Autonomous.staticObstacles);
        var toObstacle = PathfindingUtils.PointInConvexPolygons(to.getPose().getTranslation(), Autonomous.staticObstacles);
        
        if (!fromObstacle.isEmpty() && !toObstacle.isEmpty()){
            // TO and FROM are in an Obstacle, penalize the whole thing.
            R+=999*to.getPose().getTranslation().minus(from.getPose().getTranslation()).getNorm();
        } else if (toObstacle.isEmpty() && !fromObstacle.isEmpty()){
            // FROM is in an Obstacle, penalize the part of the path in the obstacle
            for (int i=0; i<fromObstacle.size(); i++){
                var start = fromObstacle.get(i);
                var end = fromObstacle.get((i+1) % fromObstacle.size());
                var intersection = VectorUtils.findIntersection(from.getPose().getTranslation(), to.getPose().getTranslation(), start, end);
                if (intersection.isPresent()){
                    R+=999*intersection.get().minus(from.getPose().getTranslation()).getNorm();
                }
            }
        } else if (fromObstacle.isEmpty() && !toObstacle.isEmpty()){
            // TO is in an Obstacle, penalize the part of the path in the obstacle.
            for (int i=0; i<toObstacle.size(); i++){
                var start = toObstacle.get(i);
                var end = toObstacle.get((i+1) % toObstacle.size());
                var intersection = VectorUtils.findIntersection(from.getPose().getTranslation(), to.getPose().getTranslation(), start, end);
                if (intersection.isPresent()){
                    R+=999*intersection.get().minus(to.getPose().getTranslation()).getNorm();
                }
            }
        } else {        
            // Neither TO or FROM are in obstacle, penalize route for passing into a static obstacle severely.
            if (PathfindingUtils.PathIntersectsPolygon(
                from.getPose().getTranslation(),
                to.getPose().getTranslation(),
                Autonomous.staticObstacles)
            ){
                R+=999999;
            };
        }

        R += VectorUtils.poseDiff(to.getPose(),from.getPose()).getTranslation().getNorm();
        return R;
    }
    public PathScorer(){
    }
}