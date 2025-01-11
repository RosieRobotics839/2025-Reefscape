// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils.pathfinding.astar;

import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

/** Add your docs here. */
public class Graph<T extends GraphNode> {
    public final List<T> nodes;
    private final Map<Integer, Set<Integer>> connections;

    public Graph(List<T> nodes, Map<Integer, Set<Integer>> connections) {
        this.nodes = nodes;
        this.connections = connections;
    }

    public T getNode(Integer id) {
        return nodes.stream()
            .filter(node -> node.getId().equals(id))
            .findFirst()
            .orElseThrow(() -> new IllegalArgumentException("No node found with ID"));
    }

    public Set<T> getConnections(T node) {
        var id = node.getId();
        var conn = connections.get(id);
        var stream= conn.stream()
            .map(this::getNode);
        var collect = stream
            .collect(Collectors.toSet());
        return collect;
    }
}
