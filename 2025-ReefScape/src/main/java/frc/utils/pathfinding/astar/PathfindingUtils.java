package frc.utils.pathfinding.astar;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import frc.utils.VectorUtils;

public class PathfindingUtils {
    public static boolean TestPointOnSide(Translation2d A, Translation2d B, Translation2d C){
      return (C.getY()-A.getY())*(B.getX()-A.getX()) > (B.getY()-A.getY())*(C.getX()-A.getX());
    }
    public static boolean VectorsIntersect (Translation2d A, Translation2d B, Translation2d C, Translation2d D){
    // Checks if the lines from A->B and C->D intersect
       return (TestPointOnSide(A, C, D) != TestPointOnSide(B, C, D)) && (TestPointOnSide( A, B, C) != TestPointOnSide(A, B, D));
    }

    /**
     * Looks for the nearest exit point from a polygon
     * @param from
     * @param polygon
     * @return
     */
    public static Translation2d nearestExit(Translation2d from, List<Translation2d> polygon){
        Translation2d nearest = from;
        double minDistance = Double.POSITIVE_INFINITY;
        for (int i=0; i<polygon.size(); i++){
            var a = polygon.get(i);
            var b = polygon.get((i+1) % polygon.size());
            var x = VectorUtils.nearestPointOnLine(from, a, b);
            var d = x.minus(from).getNorm();
            if (d < minDistance){
                nearest = x;
                minDistance = d;
            }
        }
        return nearest;
    }
    public static boolean PathIntersectsPolygon(Translation2d from, Translation2d to, List<List<Translation2d>> polygons){
        for (int i=0; i<polygons.size(); i++){
            var obstacle = polygons.get(i);
            for (int j=0; j<obstacle.size(); j++){
                var C = obstacle.get(j);
                var D = obstacle.get((j+1) % obstacle.size());
                if (PathfindingUtils.VectorsIntersect(from, to, C, D)){
                    return true;
                }
            }
        }
        return false;
    }

    /**
    * Uses PNPOLY algorithm to determine if point is in polygon by counting the number of times a horizontal line intersects the boundaries of the polygon.
    * @param point
    * @param polygon
    * @return
    */
    /*
    public static boolean PointInPolygon(Translation2d point, List<Translation2d> polygon){
        
        int i, j, c = 0;
        
        for (i = 0, j = polygon.size()-1; i < polygon.size(); j = i++) {
        if ( ((polygon.get(i).getY()>point.getY()) != (polygon.get(j).getY()>point.getY())) &&
            (point.getX() < (polygon.get(j).getX()-polygon.get(i).getX()) * (point.getY()-polygon.get(i).getY()) / (polygon.get(j).getY()-polygon.get(i).getY()) + polygon.get(i).getX()) )
            c = 1-c;
        }
        return c>0;
    };
    */

    /**
     * Calculates if a point falls within a polygon by ensuring that the point is always on the inside corner of a convex polygon.
     * This will not work correctly if the polygon is has a concave angle, but it is more efficient if the polygon is convex than other methods.
     * @param point
     * @param polygon
     * @return true if within the convex polygon
     */
    public static boolean PointInConvexPolygon(Translation2d point, List<Translation2d> polygon){
        for (int i = 0; i < polygon.size(); i++) {
            Translation2d p1 = polygon.get(i);
            Translation2d p2 = polygon.get((i + 1) % polygon.size());
            if (crossProduct(p1, p2, point) < 0) {
                return false; // Point is on the wrong side of an edge
            }
        }
        return true;
    }
        
    private static double crossProduct(Translation2d p1, Translation2d p2, Translation2d point) {
        return (p2.getX() - p1.getX()) * (point.getY() - p1.getY()) - (point.getX() - p1.getX()) * (p2.getY() - p1.getY());
    }
    
    /**
     * Returns the convex polygon which the point is inside. (Empty if none)
     * @param point
     * @param polygons list of convex polygons
     * @return the first polygon which the point is inside.
     */
    public static List<Translation2d> PointInConvexPolygons(Translation2d point, List<List<Translation2d>> polygons){
        for (int i=0; i<polygons.size(); i++){
            if (PointInConvexPolygon(point, polygons.get(i)))
                return polygons.get(i);
        }
        return new ArrayList<Translation2d>();
    }
}