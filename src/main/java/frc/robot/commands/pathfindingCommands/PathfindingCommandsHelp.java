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
public class PathfindingCommandsHelp {

  private PathPlannerPath[] activePaths;
  // Define path constraints used by the AutoBuilder for following paths.
  private static final PathConstraints pathConstraints =
      new PathConstraints(0.5, 0.4, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public PathfindingCommandsHelp(String... paths) {
    activePaths = new PathPlannerPath[paths.length];

    try {
      for (int i = 0; i < paths.length; i++) {
        activePaths[i] = PathPlannerPath.fromPathFile(paths[i]);
      }

    } catch (IOException e) {
      System.out.println("Could not load the pathplanner coral paths from file (IO exception).");
    } catch (ParseException e) {
      System.out.println("Could not load the pathplanner coral paths from file (Parse exception).");
    }
  }

  /**
   * Factory method that returns a new command to pathfind the robot to a specific depot.
   *
   * @param id the depot ID for the desired path.
   * @param left whether to use left-side paths.
   * @return a new Command instance for pathfinding.
   */
  public Command pathfindToDepotCommand(int id) {
    return AutoBuilder.pathfindThenFollowPath(activePaths[id], pathConstraints);
  }

  /**
   * Determines the closest depot path based on the robot's current position.
   *
   * @param updatedLocation the current pose of the robot.
   * @param left whether to use left-side paths.
   * @return the index of the closest depot path.
   */
  public int getClosestDepotPath(Pose2d curLocation) {
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

    for (int i = 0; i < activePaths.length; i++) {
      Pose2d testPose = activePaths[i].getPathPoses().get(0);
      double dist = testPose.getTranslation().getDistance(updatedLocation.getTranslation());
      if (dist < lowestDist) {
        lowestDist = dist;
        lowestDistID = i;
      }
    }

    return lowestDistID;
  }
}
