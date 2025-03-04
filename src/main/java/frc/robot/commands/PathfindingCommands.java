package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;

public class PathfindingCommands {
  private static PathPlannerPath[] coralPaths = new PathPlannerPath[1];
  private static boolean initialized = false;

  private static PathConstraints pathConstraints =
      new PathConstraints(0.75, 0.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // creates the array of paths from hardcode path files
  private static void init() {
    // if it is not initailized initialize it
    if (initialized) return;

    if (frc.robot.Constants.increasedPathfindingSpeed) {
      pathConstraints =
          new PathConstraints(3.5, 3, Units.degreesToRadians(540), Units.degreesToRadians(720));
    }

    initialized = true;
    coralPaths = new PathPlannerPath[6];

    // loads all the different paths from their file names
    try {
      coralPaths[0] = PathPlannerPath.fromPathFile("CoralFeed1");
      coralPaths[1] = PathPlannerPath.fromPathFile("CoralFeed2");
      coralPaths[2] = PathPlannerPath.fromPathFile("CoralFeed3");
      coralPaths[3] = PathPlannerPath.fromPathFile("CoralFeed4");
      coralPaths[4] = PathPlannerPath.fromPathFile("CoralFeed5");
      coralPaths[5] = PathPlannerPath.fromPathFile("CoralFeed6");

      for (int i = 0; i < coralPaths.length; i++) {
        System.out.println(coralPaths[i].name);
      }

      // avoids exceptions
    } catch (IOException e) {
      System.out.println("Could not load the pathplanner coral paths from the file, IO exception");
    } catch (ParseException e) {
      System.out.println(
          "Could not load the pathplanner coral paths from the file, Parse exception");
    }
  }

  // returns the command that pathfinds the robot to the specific depot id
  public static Command pathfindToDepotCommand(int depotID) {
    // initializes the REPO if it isn't already
    init();
    System.out.println("depot id: " + depotID);
    System.out.println("path with depot: " + coralPaths[depotID].getPathPoses().get(0).toString());

    for (int i = 0; i < coralPaths.length; i++) {
      System.out.println(i + " " + coralPaths[i].getPathPoses().get(0));
    }

    return AutoBuilder.pathfindThenFollowPath(coralPaths[depotID], pathConstraints);
  }

  public static Command pathfindToDepotCommand(Supplier<Integer> depotID) {
    init();

    return AutoBuilder.pathfindThenFollowPath(coralPaths[depotID.get()], pathConstraints);
  }

  public static Pose2d getDepotPose(int depotID) {
    init();
    return coralPaths[depotID].getPathPoses().get(0);
  }

  // does the same thing as pathfind with commands in between except it does a depot spot
  public static Command pathfindToDepotCommand(
      int depotID, Command commandBefore, Command commandInBetween, Command commandLast) {
    return pathfindWithCommandsInBetween(
        commandBefore, commandInBetween, commandLast, coralPaths[depotID]);
  }

  // Command before runs while the robot is getting in position and command in between runs while
  // the robot is doing the path. command after runs after everything has been finished
  public static Command pathfindWithCommandsInBetween(
      Command commandBefore, Command commandInBetween, Command commandLast, PathPlannerPath path) {
    init();

    // gets the two needed commands: pathfinding to pose and path from pose
    Command pathfindCommand =
        AutoBuilder.pathfindToPose(path.getStartingDifferentialPose(), pathConstraints);
    Command pathCommand = AutoBuilder.followPath(path);

    // a bunch of command composition to due what the function says it does(ik know it's confusing)
    return commandBefore
        .alongWith(pathfindCommand)
        .andThen(commandInBetween.alongWith(pathCommand))
        .andThen(commandLast);
  }

  // finds the closest path that goes to the depot
  public static int getClosestDepotPath(Pose2d curLocation) {

    init();

    double lowestDist = Double.MAX_VALUE;
    int lowestDistID = 0;

    // System.out.println("pose " + curLocation);

    for (int i = 0; i < coralPaths.length; i++) {
      Pose2d testPose = coralPaths[i].getPathPoses().get(0);

      double dist = testPose.getTranslation().getDistance(curLocation.getTranslation());

      if (dist < lowestDist) {
        lowestDistID = i;
        lowestDist = dist;
      }

      // System.out.println("pose: " + testPose.toString() + " dist " + dist + " id: " +
      // lowestDistID);
    }

    // System.out.println("lowest dist ID: " + lowestDistID);

    return lowestDistID;
  }

  public static Supplier<Integer> getClosestDepotPathSupplier(Supplier<Pose2d> curLocation) {

    init();

    double lowestDist = Double.MAX_VALUE;
    Integer lowestDistID = 0;

    // System.out.println("pose " + curLocation);

    for (int i = 0; i < coralPaths.length; i++) {
      Pose2d testPose = coralPaths[i].getPathPoses().get(0);

      double dist = testPose.getTranslation().getDistance(curLocation.get().getTranslation());

      if (dist < lowestDist) {
        lowestDistID = i;
        lowestDist = dist;
      }

      // System.out.println("pose: " + testPose.toString() + " dist " + dist + " id: " +
      // lowestDistID);
    }

    // System.out.println("lowest dist ID: " + lowestDistID);

    final Integer finalAns = lowestDistID;

    return () -> finalAns;
  }
}
