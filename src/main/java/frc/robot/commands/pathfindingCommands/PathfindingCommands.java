package frc.robot.commands.pathfindingCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/**
 * PathfindingCommands provides factory methods and utilities for creating and selecting pathfinding
 * commands.
 *
 * <p>Improvements Made: - Uses a factory method to always generate a new command instance. -
 * Initializes path arrays only once. - Provides a utility method to determine the closest depot
 * path based on the robot's current pose.
 */
public class PathfindingCommands {

  private static PathPlannerPath[] coralPathsLeft;
  private static PathPlannerPath[] coralPathsRight;
  private static boolean initialized = false;

  // Define path constraints used by the AutoBuilder for following paths.
  private static final PathConstraints pathConstraints =
      new PathConstraints(0.5, 0.4, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /** Initializes the path arrays if they haven't been already. */
  private static void init() {
    if (initialized) return;
    initialized = true;
    coralPathsLeft = new PathPlannerPath[6];
    coralPathsRight = new PathPlannerPath[6];

    try {
      coralPathsLeft[0] = PathPlannerPath.fromPathFile("CoralFeed1 Left");
      coralPathsLeft[1] = PathPlannerPath.fromPathFile("CoralFeed2 Left");
      coralPathsLeft[2] = PathPlannerPath.fromPathFile("CoralFeed3 Left");
      coralPathsLeft[3] = PathPlannerPath.fromPathFile("CoralFeed4 Left");
      coralPathsLeft[4] = PathPlannerPath.fromPathFile("CoralFeed5 Left");
      coralPathsLeft[5] = PathPlannerPath.fromPathFile("CoralFeed6 Left");

      coralPathsRight[0] = PathPlannerPath.fromPathFile("CoralFeed1 Right");
      coralPathsRight[1] = PathPlannerPath.fromPathFile("CoralFeed2 Right");
      coralPathsRight[2] = PathPlannerPath.fromPathFile("CoralFeed3 Right");
      coralPathsRight[3] = PathPlannerPath.fromPathFile("CoralFeed4 Right");
      coralPathsRight[4] = PathPlannerPath.fromPathFile("CoralFeed5 Right");
      coralPathsRight[5] = PathPlannerPath.fromPathFile("CoralFeed6 Right");
    } catch (IOException e) {
      System.out.println("Could not load the pathplanner coral paths from file (IO exception).");
    } catch (ParseException e) {
      System.out.println("Could not load the pathplanner coral paths from file (Parse exception).");
    }
  }

  /**
   * Factory method that returns a new command to pathfind the robot to a specific depot.
   *
   * @param depotID the depot ID for the desired path.
   * @param left whether to use left-side paths.
   * @return a new Command instance for pathfinding.
   */
  public static Command pathfindToDepotCommand(int depotID, boolean left) {
    init(); // Ensure that the paths are initialized.
    if (left) {
      return AutoBuilder.pathfindThenFollowPath(coralPathsLeft[depotID], pathConstraints);
    } else {
      return AutoBuilder.pathfindThenFollowPath(coralPathsRight[depotID], pathConstraints);
    }
  }

  /**
   * Determines the closest depot path based on the robot's current position.
   *
   * @param updatedLocation the current pose of the robot.
   * @param left whether to use left-side paths.
   * @return the index of the closest depot path.
   */
  public static int getClosestDepotPath(Pose2d curLocation, boolean left) {
    init();
    double lowestDist = Double.MAX_VALUE;
    int lowestDistID = 0;

    Pose2d updatedLocation;

    // flip the given pose if the alliance is red
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      updatedLocation =
          new Pose2d(
              FlippingUtil.flipFieldPosition(curLocation.getTranslation()),
              curLocation.getRotation());
    } else {
      updatedLocation = curLocation;
    }

    if (left) {
      for (int i = 0; i < coralPathsLeft.length; i++) {
        Pose2d testPose = coralPathsLeft[i].getPathPoses().get(0);
        double dist = testPose.getTranslation().getDistance(updatedLocation.getTranslation());
        if (dist < lowestDist) {
          lowestDist = dist;
          lowestDistID = i;
        }
      }
    } else {
      for (int i = 0; i < coralPathsRight.length; i++) {
        Pose2d testPose = coralPathsRight[i].getPathPoses().get(0);
        double dist = testPose.getTranslation().getDistance(updatedLocation.getTranslation());
        if (dist < lowestDist) {
          lowestDist = dist;
          lowestDistID = i;
        }
      }
    }
    return lowestDistID;
  }
}
