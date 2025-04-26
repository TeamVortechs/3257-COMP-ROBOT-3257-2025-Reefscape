package frc.robot.util.FieldMovement;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import java.util.TreeMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class PathfinderVortechs {

  private Supplier<Pose2d> targetPose;

  @AutoLogOutput private boolean isActive;

  private PathConstraints constraints;

  private Command pathfindingCommand;

  @AutoLogOutput private boolean isFlipping = true;

  private TreeMap<Double, Supplier<Command>> secondaryCommandMap;

  private Supplier<Pose2d> poseSupplier;

  public PathfinderVortechs(PathConstraints constraints, Supplier<Pose2d> poseSupplier) {
    this.constraints = constraints;

    targetPose = () -> new Pose2d();

    secondaryCommandMap = new TreeMap<>();

    this.poseSupplier = poseSupplier;
  }

  // sets wether or not the path automatically flips. This defaults to zero.
  public void setFlipping(boolean isFlipping) {
    this.isFlipping = isFlipping;
  }

  // sets the supplier that delegates the target pose of this helper class. USE CLOSEST POSE
  // SUPPLIER VORTECHS IT IS EASIER. Also note that this will onyl with the start command
  public void setTargetPoseSupplier(Supplier<Pose2d> targetPose) {
    this.targetPose = targetPose;
  }

  // adds a command that activates in parralel after a certain distance has been achieved
  public void addSecondaryCommand(double distance, Supplier<Command> command) {
    secondaryCommandMap.put(distance, command);
  }

  // getters

  // gets the distance between the current pose and the target pose
  public double getDistance() {
    Pose2d flippedPose;

    if (isFlipping && Constants.isFlipped()) {
      flippedPose = FlippingUtil.flipFieldPose(targetPose.get());
    } else {
      flippedPose = targetPose.get();
    }

    return poseSupplier.get().getTranslation().getDistance(flippedPose.getTranslation());
  }

  // main patthfinding commands

  // schedules the command of the pathfinder(this is the only time the starting pose gets updated)
  public void start() {
    if (isActive) {
      System.out.println(
          "TRIED TO START A NEW PATHFINDING COMMAND WHEN THE OLD IS ACTIVE. PATHFINDER VORTECHS, START");
      return;
    }

    isActive = true;

    pathfindingCommand = generatePathfindingCommand();
    pathfindingCommand.schedule();
  }

  // stops the command.
  public void stop() {
    if (isActive == false) {
      System.out.println("TRIED TO STOP A PATH WHEN IT ISN'T ACTIVE. PATHFINDINER VORTECHS, STOP");
      return;
    }

    isActive = false;
    pathfindingCommand.cancel();
  }

  private Command generatePathfindingCommand() {

    Pose2d flippedPose;

    if (isFlipping && Constants.isFlipped()) {
      flippedPose = FlippingUtil.flipFieldPose(targetPose.get());
    } else {
      flippedPose = targetPose.get();
    }

    return AutoBuilder.pathfindToPose(flippedPose, constraints)
        .alongWith(new ParallelCommandGroup(getSecondaryCommandGroup()));
  }

  // helper method that returns the hashmap of secondary commands as a parralel command group
  private Command[] getSecondaryCommandGroup() {
    Command[] commandArr = new Command[secondaryCommandMap.size()];

    // adds all the secondary commands to the array
    int index = 0;

    for (double distance : secondaryCommandMap.keySet()) {
      commandArr[index] =
          secondaryCommandMap
              .get(distance)
              .get()
              .beforeStarting(new WaitUntilCommand(() -> getDistance() < distance));

      index++;
    }

    return commandArr;
  }
}

/*

research document for layout of this class:
https://docs.google.com/document/d/13N6u-yb6LBJfHOlOZAyZdYXvdHyHThwkKrRBeyr_iXg/edit?usp=sharing
 */
