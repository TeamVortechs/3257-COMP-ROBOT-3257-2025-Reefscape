package frc.robot.util.FieldMovement.pathfindingUtil;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.CDrivetrain;
import frc.robot.util.FieldMovement.VortechsUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class PathfinderVortechs {

  // the supplier for robot pose
  private Supplier<Pose2d> poseSupplier;

  // wether or not the pathfinding command is active
  @AutoLogOutput private boolean isActive = false;

  // the current rendition of the pathfinding command
  private Command pathfindingCommand;

  private PathConstraints constraints;

  // the target pose of the pathfinder
  private Pose2d targetPose;

  public PathfinderVortechs(PathConstraints constraints, Supplier<Pose2d> poseSupplier) {
    this.constraints = constraints;

    this.poseSupplier = poseSupplier;

    // initializes the non-initialized part of the class
    targetPose = new Pose2d();
    pathfindingCommand = new InstantCommand();
  }

  // pathfinding manager commands
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

  // helper command that does everything and stops the commadn when the drivetrain reaches position
  public Command startPath(Pose2d targetPose) {
    // stops old routine
    return new InstantCommand(() -> stop())
        // sets the target pose
        .andThen(new InstantCommand(() -> setPose(targetPose)))
        // starts the new routine
        .andThen(new InstantCommand(() -> start()))
        // ending logic
        .andThen(new WaitUntilCommand(() -> isOnTarget()))
        .andThen(new InstantCommand(() -> stop()));
    // stops hogging drivetrain
  }

  // util setters
  public void setPose(Pose2d targetPose) {
    this.targetPose = targetPose;
  }

  // getters
  // gets the distance between the current pose and the target pose
  public double getDistance() {
    return poseSupplier.get().getTranslation().getDistance(targetPose.getTranslation());
  }

  public double getRotationDifferenceRAD() {
    return poseSupplier.get().getRotation().getRadians() - targetPose.getRotation().getRadians();
  }

  public boolean isOnTarget() {
    return VortechsUtil.isInTolerance(getDistance(), CDrivetrain.TOLERANCE_DIST)
        && VortechsUtil.isInTolerance(getRotationDifferenceRAD(), CDrivetrain.TOLERANCE_ROT);
  }

  // helper commands
  private Command generatePathfindingCommand() {

    Pose2d flippedPose;

    if (Constants.isFlipped()) {
      flippedPose = FlippingUtil.flipFieldPose(targetPose);
    } else {
      flippedPose = targetPose;
    }

    return AutoBuilder.pathfindToPose(flippedPose, constraints);
  }
}

/*

research document for layout of this class:
https://docs.google.com/document/d/13N6u-yb6LBJfHOlOZAyZdYXvdHyHThwkKrRBeyr_iXg/edit?usp=sharing
 */

 /*
 get the closest one from a pipeline
  */
