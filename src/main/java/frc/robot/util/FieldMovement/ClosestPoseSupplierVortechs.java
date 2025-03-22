package frc.robot.util.FieldMovement;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

public class ClosestPoseSupplierVortechs {
  private Pose2d[] targetPoses;
  private Supplier<Pose2d> drivetrainPoseSupplier;

  public ClosestPoseSupplierVortechs(
      Supplier<Pose2d> drivetrainPoseSupplier, Pose2d... targetPoses) {
    this.drivetrainPoseSupplier = drivetrainPoseSupplier;
    this.targetPoses = targetPoses;
  }

  public Pose2d getClosestPose() {
    double closestDistance = Double.MAX_VALUE;
    Pose2d closestPose = new Pose2d();

    for (int i = 0; i < targetPoses.length; i++) {
      double testDist =
          drivetrainPoseSupplier
              .get()
              .getTranslation()
              .getDistance(targetPoses[i].getTranslation());

      if (testDist < closestDistance) {
        closestDistance = testDist;
        closestPose = targetPoses[i];
      }
    }

    return closestPose;
  }
}
