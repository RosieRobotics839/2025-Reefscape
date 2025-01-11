// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils.pathfinding.astar;

import java.util.ArrayList;
import java.util.List;
import java.util.TreeSet;
import java.util.stream.Collectors;

/** Add your docs here. */
public class RouteFinder<T extends GraphNode> {
    private final Graph<T> graph;
    private final Scorer<T> nextNodeScorer;
    private final Scorer<T> targetScorer;

    public RouteFinder(Graph<T> graph, Scorer<T> nextNodeScorer, Scorer<T> targetScorer) {
        this.graph = graph;
        this.nextNodeScorer = nextNodeScorer;
        this.targetScorer = targetScorer;
    }

    public T bestEntryPoint(T fpose, T to){
        List<T> nodes = new ArrayList<>();
        List<Double> scores = new ArrayList<>();
        graph.nodes.forEach((node) -> {

            double score = nextNodeScorer.computeCost(fpose,node) + targetScorer.computeCost(node, to);
            RouteNode<T> next = new RouteNode<T>(node, null, 0d, targetScorer.computeCost(fpose, to));
            if (nodes.isEmpty() || score < scores.get(0)) {
                next.setRouteScore(score);
                nodes.add(0, next.getCurrent());
                scores.add(0, score);
            }
        });
        return nodes.get(0);
    }

    public List<T> findRoute(T from, T to) {
        List<RouteNode<T>> allNodes = graph.nodes.stream().map(n-> new RouteNode<T>(n)).collect(Collectors.toList()); 

        TreeSet<RouteNode<T>> openSet = new TreeSet<RouteNode<T>>();

        RouteNode<T> start = allNodes.get(from.getId());
        start.setRouteScore(0);
        openSet.add(start);

        while (!openSet.isEmpty()) {
            RouteNode<T> next = openSet.pollFirst();
            if (next.getCurrent().equals(to)) {

                List<T> route = new ArrayList<>();
                RouteNode<T> current = next;
                do {
                    route.add(0, current.getCurrent());
                    if(current.getPrevious() == null){
                        break;
                    }
                    current = allNodes.get(current.getPrevious().getId());
                } while (current != null);

                return route;
            }

            graph.getConnections(next.getCurrent()).forEach(connection -> {
                double newScore = next.getRouteScore() + nextNodeScorer.computeCost(next.getCurrent(), connection);
                RouteNode<T> nextNode = allNodes.get(connection.getId());
                if (nextNode != null && nextNode.getRouteScore() > newScore) {
                    nextNode.setPrevious(next.getCurrent());
                    nextNode.setRouteScore(newScore);
                    nextNode.setEstimatedScore(newScore + targetScorer.computeCost(connection, to));
                    openSet.add(nextNode);
                }
            });
        }

        return new ArrayList<>();
        // No route found
    }
}