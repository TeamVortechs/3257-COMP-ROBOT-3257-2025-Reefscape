package frc.robot.util.FieldMovement;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.function.Supplier;

public class ClosestPoseSupplierVortechs {
  private List<Pose2d>[] targetPoses;
  private Supplier<Pose2d> drivetrainPoseSupplier;
  private int index = 0;

  public ClosestPoseSupplierVortechs(
      Supplier<Pose2d> drivetrainPoseSupplier, List<Pose2d>... targetPoses) {
    this.drivetrainPoseSupplier = drivetrainPoseSupplier;
    this.targetPoses = targetPoses;
  }

  public void setIndex(int index) {
    this.index = index;
  }

  public int getIndex() {
    return index;
  }

  public Pose2d getClosestPose() {
    double closestDistance = Double.MAX_VALUE;
    Pose2d closestPose = new Pose2d();

    for (Pose2d targetPose : targetPoses[index]) {
      double testDist =
          drivetrainPoseSupplier.get().getTranslation().getDistance(targetPose.getTranslation());

      if (testDist < closestDistance) {
        closestDistance = testDist;
        closestPose = targetPose;
      }
    }

    return closestPose;
  }
}
