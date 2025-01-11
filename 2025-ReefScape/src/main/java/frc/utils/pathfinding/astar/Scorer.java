// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils.pathfinding.astar;

/** Add your docs here. */
public interface Scorer<T extends GraphNode> {
    double computeCost(T from, T to);
}