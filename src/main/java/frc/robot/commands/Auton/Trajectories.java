package frc.robot.commands.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import java.util.List;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Vector;
import java.util.stream.Collectors;

public class Trajectories {

    /**
     * Trajectory Configs 
     */
    TrajectoryConfig testConfig = new TrajectoryConfig(3, 1.5);

    public Trajectories() {
        testConfig.setStartVelocity(0);
        testConfig.setEndVelocity(0);
      //  testConfig.setReversed(true);
    }

    /**
     * Most likely deprecated with the use of new GUI Path Planner. Use CSV files instead!
     * @param trajectoryID - Available: [1 - Test Trajectory] [2 - Barrel Race Path]
     * @param currentPose - The current position(x, y, z) of the robot in the object Pose2d
     * @return
     */
    public Trajectory generateAutoTrajectoryFromCurrentPose(int trajectoryID, Pose2d currentPose){
        switch (trajectoryID){
            case 1:
            return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(new Pose2d(), new Pose2d(Units.feetToMeters(10), 0, Rotation2d.fromDegrees(0))), 
                testConfig
            );
            case 2:
            return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(Units.feetToMeters(12), 0, Rotation2d.fromDegrees(0))), 
                testConfig
            );
            case 3:
            return TrajectoryGenerator.generateTrajectory(
                Arrays.asList(new Pose2d(Units.feetToMeters(12), 0, Rotation2d.fromDegrees(0)), new Pose2d(Units.feetToMeters(12), Units.feetToMeters(-5), Rotation2d.fromDegrees(0))), 
                testConfig
            );

            //new Pose2d(1.468, -0.9, Rotation2d.fromDegrees(0)), 
            //new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)), new Pose2d(4.05, 0, Rotation2d.fromDegrees(0)))
            default:
                return TrajectoryGenerator.generateTrajectory(
                    Arrays.asList(currentPose, new Pose2d(0, Units.feetToMeters(5), Rotation2d.fromDegrees(0))), // Makes the robot go 5 feet in y direction on autonomousInit();
                    testConfig
                );
        }
    }

     /**
     * Loads a Trajectory JSON into a Trajectory object
     * @param pathToJSON - File path to JSON of the trajectory
     * @return Trajectory loaded from given JSON
     */
    public Trajectory loadTrajectoryFromJSON(String pathToJSON){
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathToJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + pathToJSON, ex.getStackTrace());
        }

        return trajectory;
    }

    /**
     * Loads a trajectory from a 2D double array
     * @param path - The double array to load from
     * @return
     */
    public Trajectory loadTrajectoryFromPathArray(Double[][] path){
        List<Trajectory.State> states = new Vector<Trajectory.State>();


        for (Double[] d : path){
            states.add(new Trajectory.State(d[0], d[1], d[2], new Pose2d(d[3], d[4], Rotation2d.fromDegrees(d[5])), 1/d[6]));
        }


        return new Trajectory(states);
    }

    private Trajectory[] loadTrajectoryFromPathCSV(String path) throws IOException {
        List<Trajectory.State> states = new Vector<Trajectory.State>();
        List<Trajectory.State> headingStates = new Vector<Trajectory.State>();

        // path should be something like paths/ExamplePath.csv
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
        //Path trajectoryPath = Path.of("C:/Users/jacka/Documents/Robotics/PathPlanner/ExamplePath.csv");

        List<String[]> collect =
          Files.lines(trajectoryPath).map(line -> line.split(",")).collect(Collectors.toList());

        // Path Planner output format: (t, v, a, x, y, hh, r, h)
        int c = 0;
        double changeInRad;
        double timeStep = 0.01;
        double angularVelocity;

        for (String[] s : collect){
            if (s[6] == "Infinity"){
                states.add(new Trajectory.State(Double.parseDouble(s[0]), Double.parseDouble(s[1]), Double.parseDouble(s[2]), new Pose2d(Double.parseDouble(s[3]), -Math.sqrt(2)*Double.parseDouble(s[4]), Rotation2d.fromDegrees(-1*Double.parseDouble(s[5]))), 1/Double.POSITIVE_INFINITY));
            } else {
                states.add(new Trajectory.State(Double.parseDouble(s[0]), Double.parseDouble(s[1]), Double.parseDouble(s[2]), new Pose2d(Double.parseDouble(s[3]), -Math.sqrt(2)*Double.parseDouble(s[4]), Rotation2d.fromDegrees(-1*Double.parseDouble(s[5]))), 1/Double.parseDouble(s[6])));
            }
            if (c == 0){
                headingStates.add(new Trajectory.State(Double.parseDouble(s[0]), 0, 0, new Pose2d(Double.parseDouble(s[3]), -1*Double.parseDouble(s[4]), Rotation2d.fromDegrees(Double.parseDouble(s[7]))), 0));
            } else {
                changeInRad = Math.toRadians(Math.abs(Double.parseDouble(s[5])) - Math.abs(Double.parseDouble(collect.get(c-1)[5])));
                angularVelocity = changeInRad/timeStep;
                headingStates.add(new Trajectory.State(Double.parseDouble(s[0]), angularVelocity, 0, new Pose2d(Double.parseDouble(s[3]), -1*Double.parseDouble(s[4]), Rotation2d.fromDegrees(Double.parseDouble(s[7]))), 0));
                //System.out.println(changeInRad);
            }
            c++;
        
        }

        Trajectory[] trajectoryArray = new Trajectory[2];

        trajectoryArray[0] = new Trajectory(states);
        trajectoryArray[1] = new Trajectory(headingStates);
            
        return trajectoryArray;
    }

    /**
     * Loads a Trajectory from a csv file in the robots deploy directory. 
     * @param path - The path to the csv file. Should be like: paths/example.csv
     * @return Trajectory[] from given state points in the csv file. [0] = Trajectory to follow. [1] = Heading Trajectory
     */
    public Trajectory[] getTrajectoryFromCSV(String path){
        try {
            return loadTrajectoryFromPathCSV(path);
        } catch (IOException e) {
            System.out.println("Error loading path from csv file: " + path);
            System.out.println("Trajectory supplied will be empty!");
            DriverStation.reportError("Error loading path from csv file: " + path, e.getStackTrace());
            Trajectory[] nullTraj = new Trajectory[2];
            return nullTraj;
        }
    }

    /**
     * Test method to run methods outside of the entire program. Allows for testing trajectories
     */
    public static void main(String[] args) {
    }
}