// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils.pathfinding.astar;

/** Add your docs here. */
import java.util.StringJoiner;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldPose implements GraphNode {
    private Integer id;
    private final String name;
    private Pose2d pose;

    public FieldPose(Integer id, String name, Pose2d pose) {
        this.id = id;
        this.name = name;
        this.pose = pose;
    }

    @Override
    public Integer getId() {
        return id;
    }

    public void setId(Integer id) {
        this.id = id;
    }

    public String getName() {
        return name;
    }

    public Pose2d getPose(){
        return pose;
    }
    
    public void setPose(Pose2d pose){
        this.pose = pose;
    }

    @Override
    public String toString() {
        return new StringJoiner(", ", FieldPose.class.getSimpleName() + "[", "]").add("id='" + id.toString() + "'")
            .add("name='" + name + "'").add("x=" + pose.getX()).add("y=" + pose.getY()).toString();
    }
}