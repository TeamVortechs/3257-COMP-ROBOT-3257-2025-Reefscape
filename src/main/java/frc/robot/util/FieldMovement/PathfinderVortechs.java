package frc.robot.util.FieldMovement;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class PathfinderVortechs {

  private Supplier<Pose2d> targetPose;

  @AutoLogOutput private boolean isActive;

  private PathConstraints constraints;

  private Command pathfindingCommand;

  @AutoLogOutput private boolean isFlipping = true;

  private Supplier<Pose2d> targetPoseSupplier;

  public PathfinderVortechs(PathConstraints constraints, Supplier<Pose2d> poseSupplier) {
    this.constraints = constraints;

    targetPose = () -> new Pose2d();

    this.targetPoseSupplier = poseSupplier;
  }

  public void setFlipping(boolean isFlipping) {
    this.isFlipping = isFlipping;
  }

  public void setTargetPoseSupplier(Supplier<Pose2d> targetPose) {
    this.targetPose = targetPose;
  }

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

    if (isFlipping && DriverStation.getAlliance().get() == Alliance.Red) {
      flippedPose = FlippingUtil.flipFieldPose(targetPose.get());
    } else {
      flippedPose = targetPose.get();
    }

    return AutoBuilder.pathfindToPose(flippedPose, constraints)
        .until(
            () ->
                targetPoseSupplier.get().getTranslation().getDistance(flippedPose.getTranslation()) < 0.4)
        .andThen(new InstantCommand(() -> stop()))
        .andThen(new PrintCommand("REACHED POINT"));
  }
}

/*

research document for layout of this class:
https://docs.google.com/document/d/13N6u-yb6LBJfHOlOZAyZdYXvdHyHThwkKrRBeyr_iXg/edit?usp=sharing
 */
