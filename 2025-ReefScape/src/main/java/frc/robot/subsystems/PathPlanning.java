// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.utils.VectorUtils;
import frc.utils.pathfinding.astar.FieldPose;
import frc.utils.pathfinding.astar.ScoreHeuristic;
import frc.utils.pathfinding.astar.Graph;
import frc.utils.pathfinding.astar.PathScorer;
import frc.utils.pathfinding.astar.RouteFinder;

/** Add your docs here. */
public class PathPlanning {

    static PathPlanning instance = new PathPlanning();
    public static PathPlanning getInstance(){
        return instance;
    }
  
    FieldPose m_current = new FieldPose(-1, "current", PoseEstimator.getInstance().m_finalPose);
    FieldPose m_destination = new FieldPose(-2, "destination", PoseEstimator.getInstance().m_finalPose);

    public List<FieldPose> orignodes = new ArrayList<FieldPose>();
    Map<Integer, Set<Integer>> map = new HashMap<Integer, Set<Integer>>();
    PathScorer nextNodeScorer = new PathScorer();
    ScoreHeuristic targetScorer = new ScoreHeuristic();
    public Graph<FieldPose> fieldgraph;
    public volatile List<FieldPose> nodes = new ArrayList<FieldPose>();

    FieldPose pose;

    private Set<Integer> connection(Integer node, int[] nodes){

        Set<Integer> connect = new HashSet<Integer>();
        for (Integer n : nodes){
            connect.add(n);
        }
        return connect;
    }

    private void addNode(FieldPose pose, int[] connectedNodes){
        orignodes.add(pose);
        map.put(pose.getId(), connection(pose.getId(), connectedNodes));
    }
    public PathPlanning(){
        fromChorFile("autopointgraph.json");
        calcFieldGraph();
    }

    public void calcFieldGraph(){
        nodes.clear();
        if (FlightStick.m_blueAlly){
            nodes.addAll(this.orignodes);
        } else {
            double halffield = Vision.getInstance().aprilTagFieldLayout.getFieldLength()/2.0;
            nodes.addAll(this.orignodes.stream().map(f->new FieldPose(f.getId(), f.getName(), new Pose2d(new Translation2d(halffield+(halffield-f.getPose().getX()),f.getPose().getY()),null))).collect(Collectors.toList()));
        }

        int nnodes = nodes.size();

        GraphOnField2d(nodes);

        // Add current and destination nodes
        m_current.setId(nnodes);
        m_destination.setId(nnodes+1);
        nodes.add(m_current);
        nodes.add(m_destination);

        if (map.size() == nnodes) {
            // Add current location as node with connections to every node
            Set<Integer> conn = IntStream.range(0,nnodes).boxed().collect(Collectors.toSet());
            conn.add(m_destination.getId());
            map.put(m_current.getId(), conn);
            
            // Add destination location as node with connections to every node
            conn = IntStream.range(0,nnodes).boxed().collect(Collectors.toSet());
            conn.add(m_current.getId());
            map.put(m_destination.getId(), conn);

            for (int i=0; i<nnodes; i++){
                map.get(i).add(m_current.getId());
                map.get(i).add(m_destination.getId());
            }
        }

        fieldgraph = new Graph<FieldPose>(nodes, map);
    }

    public void GraphOnField2d(List<FieldPose> fieldnodes){
        List<Pose2d> poses = fieldnodes.stream().map(a->new Pose2d(a.getPose().getTranslation(),new Rotation2d(0))).collect(Collectors.toList());
        PoseEstimator.getInstance().m_field.getObject("GraphNodes").setPoses(poses);
    }

    public Graph<FieldPose> fromChorFile(String pathName) {
        try (BufferedReader br =
            new BufferedReader(
                new FileReader(
                    new File(
                        Filesystem.getDeployDirectory(), pathName)))) {
        StringBuilder fileContentBuilder = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) {
            fileContentBuilder.append(line);
        }

        String fileContent = fileContentBuilder.toString();
        JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

        JSONObject nodegraphjson = (JSONObject) json.get("NodeGraph");
        JSONArray nodesjson = (JSONArray) nodegraphjson.get("Nodes");

        for (Integer i=0; i < nodesjson.size(); i++){
            JSONObject waypointjson = (JSONObject) nodesjson.get(i);
            JSONArray connectionsjson = (JSONArray) waypointjson.get("connections");

            Pose2d pose = new Pose2d(
                ((Number) waypointjson.get("x")).doubleValue(),
                ((Number) waypointjson.get("y")).doubleValue(),
                null);
            FieldPose fieldpose = new FieldPose(i,i.toString(),pose);

            addNode(fieldpose, JSonArray2IntArray(connectionsjson));
        }

        return fieldgraph;
        } catch (Exception e) {
        e.printStackTrace();
        return null;
        }
    }

    public static int[] JSonArray2IntArray(JSONArray jsonArray){
        int[] intArray = new int[jsonArray.size()];
        for (int i = 0; i < jsonArray.size(); ++i) {
            intArray[i] = ((Number)jsonArray.get(i)).intValue();
        }
        return intArray;
    }

    public void clearNavQueue(){
        DriveTrain.getInstance().m_poseQueue.clear();
        DriveTrain.getInstance().PublishPoseQueue();
    }

    public void flipFieldGraph(boolean isBlueAlly){
        if (isBlueAlly){

        }
    }

    public void navigateTo(Pose2d to){
        Pose2d from;
        if (DriveTrain.getInstance().m_poseQueue.size() == 0){
            from = PoseEstimator.getInstance().m_finalPose;
        } else {
            from = DriveTrain.getInstance().m_poseQueue.getLast();
        }
        navigateTo(to, from);
    }

    public void navigateTo(Pose2d to, Pose2d from){
        m_current.setPose(from);
        m_destination.setPose(to);

        // Otherwise find the best route using A*
        RouteFinder<FieldPose> routeFinder = new RouteFinder<FieldPose>(fieldgraph, nextNodeScorer, targetScorer);
        List<FieldPose> route = routeFinder.findRoute(m_current, m_destination);
        for (int i=1; i<route.size()-1; i++){
            DriveTrain.getInstance().m_poseQueue.offer(new Pose2d(route.get(i).getPose().getTranslation(),null));
        }
        if (DriveTrain.getInstance().m_poseQueue.isEmpty()){
            DriveTrain.getInstance().m_poseQueueStart = from;
        }
        DriveTrain.getInstance().m_poseQueue.offer(to);
        DriveTrain.getInstance().PublishPoseQueue();
    }

    public void navigateCloseTo(Pose2d target, double distance, double angleTol){
        navigateCloseTo(target, distance, angleTol, false);
    }
    public void navigateCloseTo(Pose2d target, double distance, double angleTol, boolean within){
        if (within && VectorUtils.isInDistanceAndAngle(PoseEstimator.getInstance().m_finalPose, target, distance, angleTol)){
            return;
        }

        DriveTrain dt = DriveTrain.getInstance();
        LinkedList<Pose2d> orig_q = new LinkedList<Pose2d>();

        orig_q.addAll(dt.m_poseQueue);
        navigateTo(target);
        if ((dt.m_poseQueue.size()-orig_q.size()) == 0){
            // nothing planned, already there?
            return;
        }

        if ((dt.m_poseQueue.size()-orig_q.size()) == 1 && orig_q.size() == 0){
            dt.m_poseQueue = orig_q;
            navigateTo(VectorUtils.closestPoseAtDistance(PoseEstimator.getInstance().m_finalPose, target, distance, angleTol));
            return;
        }

        // Remove the end nav point and replace it with one at the distance and within angle tolerance.
        dt.m_poseQueue.pollLast();
        Pose2d closestPose = VectorUtils.closestPoseAtDistance(dt.m_poseQueue.peekLast(), target, distance, angleTol);
        dt.m_poseQueue = orig_q;
        navigateTo(closestPose);

    }
    
    /**
     * Provides a way to get the Pose that the robot should be in relative to an apriltag
     * @param id Apriltag ID
     * @param translation to move relative to the apriltag, if looking at it.
     * @param radians angle to rotate the robot applied after translating it.
     * @return
     */
    public static Pose2d AprilTagAtDistance(int id, Translation2d translation, double radians) {
        Pose2d tagPose = Vision.getInstance().aprilTagFieldLayout.getTagPose(id).get().toPose2d();
        return PoseAtDistance(tagPose, translation, radians);
    }

    public static Pose2d PoseAtDistance(Pose2d pose, Translation2d translation, double radians){
        Pose2d result = new Pose2d(
            pose.getTranslation().plus(translation.rotateBy(new Rotation2d(pose.getRotation().getRadians()+Math.PI))),
            pose.getRotation().plus(new Rotation2d(radians+Math.PI))
        );
        return result;
    }
    public static Pose2d PoseAtDistance(Pose2d pose, Translation2d translation){
        return PoseAtDistance(pose, translation, 0);
    }
    
    public static Pose2d AprilTagAtDistance(int id, Translation2d translation) {
        return AprilTagAtDistance(id, translation, 0);
    }

    public static Pose2d AprilTagAtDistance(int id, double distance) {
        return AprilTagAtDistance(id, new Translation2d(distance, 0), 0);
    }
    
    public static Pose2d AprilTagAtDistance(int id, double distance, double radians) {
        return AprilTagAtDistance(id, new Translation2d(distance, 0), radians);
    }
}
