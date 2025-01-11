// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils.pathfinding.astar;

/** Add your docs here. */
class RouteNode<T extends GraphNode> implements Comparable<RouteNode<T>> {
    private final T current;
    private T previous;
    private double routeScore;
    private double estimatedScore;

    RouteNode(T current) {
        this(current, null, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    RouteNode(T current, T previous, double routeScore, double estimatedScore) {
        this.current = current;
        this.previous = previous;
        this.routeScore = routeScore;
        this.estimatedScore = estimatedScore;
    }

    public T getCurrent(){
        return current;
    }

    public void setPrevious(T _previous){
        previous = _previous;
    }
     
    public T getPrevious(){
        return previous;
    }

    public void setRouteScore(double _score){
        routeScore = _score;
    }

    public double getRouteScore(){
        return routeScore;
    }

    public void setEstimatedScore(double _score){
        estimatedScore = _score;
    }
    
    public double getEstimatedScore(){
        return estimatedScore;
    }

    @Override
    public int compareTo(RouteNode<T> other) {
    if (this.estimatedScore > other.estimatedScore) {
        return 1;
    } else if (this.estimatedScore < other.estimatedScore) {
        return -1;
    } else {
        return 0;
    }
}
}